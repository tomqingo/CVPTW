# include <iostream>
# include <fstream>
# include <sstream>
# include "string.h"
# include <vector>
# include <limits.h>
# include "math.h"
# include <time.h>
using namespace std;

/*
int main()
{    
    ifstream infile;
    string filename = "/data/ssd/qluo/ILP_homework/instances/Set 1/c101.txt";
    infile.open(filename);

    int capacity;
    vector<vector<int>> coordinatecol;
    vector<vector<int>> permitperiodcol;
    vector<int> visittime;
    vector<int> demand;
    string line;
    getline(infile, line);
    capacity = stoi(line);

    cout<<capacity<<endl;

    int i = 0;
    while(getline(infile, line))
    {
        istringstream iss(line);
        string subline;
        getline(iss, subline, '\t');
        int id = stoi(subline);
        //coordinate
        vector<int> coordinate;
        getline(iss, subline, '\t');
        cout<<subline<<endl;
        coordinate.push_back(stoi(subline));
        getline(iss, subline, '\t');
        coordinate.push_back(stoi(subline));
        coordinatecol.push_back(coordinate);
        //demand
        getline(iss, subline, '\t');
        demand.push_back(stoi(subline));
        //time period
        vector<int> permitperiod;
        getline(iss, subline, '\t');
        permitperiod.push_back(stoi(subline));
        getline(iss, subline, '\t');
        permitperiod.push_back(stoi(subline));
        permitperiodcol.push_back(permitperiod);
        //visit time
        getline(iss, subline, '\t');
        visittime.push_back(stoi(subline));

        cout<<id<<' '<<coordinatecol[i][0]<<' '<<coordinatecol[i][1]<<demand[i]<<' '<<permitperiodcol[i][0]<<' '<<permitperiodcol[i][1]<<' '<<visittime[i]<<endl;
        i++;
    }
    return 0;
}
*/
void calDistanceMatrix(vector<vector<int>> coordinatecol, vector<vector<double>>& distmatrix);
void greedyheuristicSearch(vector<vector<double>> distmatrix, vector<vector<double>> permitperiodcol, vector<double> visittime, vector<int> demand, int capacity);
void route_initialize(vector<int>& route, vector<vector<double>> permitperiodcol, vector<bool>& visited);
vector<double> FindBestInsertion(vector<int> route, int insert_id, vector<vector<double>> distmatrix, vector<vector<double>> permitperiodcol, vector<double> visittime);
void calInitialTiming(vector<int> route, vector<vector<double>> distmatrix, vector<double> visittime, vector<vector<double>> permitperiodcol, vector<double>& service_beginning_time);
bool checkCapacity(vector<vector<int>> routecol, vector<int> demand, int capacity);
bool checkArrivalTime(vector<vector<int>> routecol, vector<vector<double>> distmatrix, vector<double> visittime, vector<vector<double>> permitperiodcol);
double calTotalDistance(vector<vector<int>> routecol, vector<vector<double>> distmatrix);
bool checkArrivalTimesub(vector<int> route, int insert_obj, int insert_place, vector<vector<double>> distmatrix, vector<double> visittime, vector<vector<double>> permitperiodcol);
void TabuRefinement(vector<vector<int>>& routecol, vector<vector<double>> distmatrix, vector<vector<double>> permitperiodcol, vector<double> visittime, vector<int> demand, int capacity);
int main(int argc, char *argv[])
{
    string filename = argv[2];
    ifstream infile(argv[2]);
    //string filename = "/data/ssd/qluo/ILP_homework/instances/Set 1/c109.txt";
    //string filename = "/data/ssd/qluo/ILP_homework/instances/Set 6/rc208.txt";
    //infile.open(filename);
    int capacity;
    vector<int> idcol;
    vector<vector<int>> coordinatecol;
    vector<vector<double>> permitperiodcol;
    vector<vector<double>> distmatrix;
    vector<double> visittime;
    vector<int> demand;
    string line;
    getline(infile, line);
    //cout<<line<<endl;
    capacity = stoi(line);

    //cout<<capacity<<endl;

    int i = 0;
    while(getline(infile, line))
    {
        int j = 0;
        string subline;
        vector<int> coordinate;
        vector<double> permitperiod;
        //cout<<line<<endl;
        for(int k=0; k<line.size(); k++)
        {
            if(line[k]=='\t'||line[k]==' ')
            {
                if(subline.size()!=0)
                {
                    //cout<<subline<<endl;
                    switch(j)
                    {
                        case 0: 
                            idcol.push_back(stoi(subline));
                            break;
                        case 1:
                            coordinate.push_back(stoi(subline));
                            break;
                        case 2:
                            coordinate.push_back(stoi(subline));
                            coordinatecol.push_back(coordinate);
                            coordinate.clear();
                            break;
                        case 3:
                            demand.push_back(stoi(subline));
                            break;
                        case 4:
                            permitperiod.push_back(stod(subline));
                            break;
                        case 5:
                            permitperiod.push_back(stod(subline));
                            permitperiodcol.push_back(permitperiod);
                            permitperiod.clear();
                            break;
                        case 6:
                            visittime.push_back(stod(subline));
                            break;
                        default:
                            break;
                    }
                    j++;
                    subline.clear();
                }
                
                continue;
            }
            subline.push_back(line[k]);
        }

        if(subline.size()!=0)
            visittime.push_back(stoi(subline));

        //cout<<idcol[i]<<' '<<coordinatecol[i][0]<<' '<<coordinatecol[i][1]<<' '<<demand[i]<<' '<<permitperiodcol[i][0]<<' '<<permitperiodcol[i][1]<<' '<<visittime[i]<<endl;
        i++;

    }
    infile.close();
    clock_t start,end;
    start = clock();
    calDistanceMatrix(coordinatecol,distmatrix);
    greedyheuristicSearch(distmatrix,permitperiodcol,visittime,demand,capacity);
    end = clock();
    cout<<"The running time of the algorithm:"<<(double)(end-start)/CLOCKS_PER_SEC<<endl;
    return 0;
}

void calDistanceMatrix(vector<vector<int>> coordinatecol, vector<vector<double>>& distmatrix)
{
    for(int i=0; i<coordinatecol.size(); i++)
    {
        vector<double> distrow;
        for(int j=0; j<coordinatecol.size(); j++)
        {
            double dist = sqrt((pow(coordinatecol[i][0]-coordinatecol[j][0],2)+pow(coordinatecol[i][1]-coordinatecol[j][1],2))*1.0);
            distrow.push_back(dist);
        }
        distmatrix.push_back(distrow);
    }
}

void greedyheuristicSearch(vector<vector<double>> distmatrix, vector<vector<double>> permitperiodcol, vector<double> visittime, vector<int> demand, int capacity)
{
    //hyperparameter
    cout<<"Greedy Search Begins!"<<endl;
    double lambda = 1.0;
    vector<vector<int>> routecol;
    vector<bool> visited;

    for(int i=0; i<visittime.size(); i++)
        visited.push_back(false);
    visited[0] = true;

    bool all_visit_flag = false;

    //int row = 0;
    while(!all_visit_flag)
    {
        vector<int> route;
        //initialization
        route_initialize(route, permitperiodcol, visited);
        /*
        if(row ==0)
        {
            for(int i=0; i<route.size(); i++)
                cout<<route[i]<<' ';
            cout<<endl;
        }
        */
        int total_demand = demand[route[1]];
        bool overflow_flag = false;
        while(!overflow_flag)
        {
            double MAX_COST = -INT_MAX+1;
            int max_cost_id;
            int insert_id;
            int routeLength_prev = route.size();
            for(int i=1; i<visittime.size(); i++)
            {
                if(!visited[i]&&(total_demand+demand[i]<=capacity))
                {
                    vector<double> insertresult;
                    insertresult = FindBestInsertion(route, i, distmatrix, permitperiodcol, visittime);
                    if(insertresult.size()==0)
                        continue;
                    /*
                    if(!checkArrivalTimesub(route, i, int(insertresult[1]), distmatrix, visittime, permitperiodcol))
                        continue;
                    */
                    
                    if((lambda*distmatrix[0][i]-insertresult[0])>MAX_COST)
                    {
                        MAX_COST = lambda*distmatrix[0][i]-insertresult[0];
                        max_cost_id = i;
                        insert_id = int(insertresult[1]);
                    }                        
                }
            }

            if(MAX_COST!=-INT_MAX+1)
            {
                /*
                if(row == 0)
                    cout<<max_cost_id<<' '<<insert_id<<endl;
                */
                route.insert(route.begin()+insert_id, max_cost_id);
                total_demand += demand[max_cost_id];
                visited[max_cost_id] = true;
            }

            //cout<<route.size()<<' '<<routeLength_prev<<endl;
            if(route.size()==routeLength_prev)
                overflow_flag = true;
        }
        routecol.push_back(route);
        bool flag_sub = true;
        for(int i=0; i<visited.size(); i++)
            flag_sub = flag_sub && visited[i];
        if(flag_sub)
            all_visit_flag = true;
        //row++;
    }

    //check time, capacity and calculate total distance
    if(!checkArrivalTime(routecol,distmatrix,visittime,permitperiodcol))
        cout<<"The arrival time surpasses the deadline"<<endl;
    if(!checkCapacity(routecol,demand,capacity))
        cout<<"The capacity of the car is surpassed"<<endl;
    double total_distance = 0.0;
    total_distance = calTotalDistance(routecol,distmatrix);

    //visualization
    cout<<"The routes are as follows:"<<endl;
    for(int i=0; i<routecol.size(); i++)
    {
        for(int j=0; j<routecol[i].size(); j++)
            cout<<routecol[i][j]<<' ';
        cout<<endl;
    }
    cout<<"Total Distance: "<<total_distance<<endl;

    //tabu refinement
    cout<<"Tabu Refinement Begins!"<<endl;
    TabuRefinement(routecol, distmatrix, permitperiodcol, visittime, demand, capacity);
    //check time, capacity and calculate total distance
    if(!checkArrivalTime(routecol,distmatrix,visittime,permitperiodcol))
        cout<<"The arrival time surpasses the deadline"<<endl;
    if(!checkCapacity(routecol,demand,capacity))
        cout<<"The capacity of the car is surpassed"<<endl;
    total_distance = calTotalDistance(routecol,distmatrix);

    //visualization
    cout<<"The routes are as follows:"<<endl;
    for(int i=0; i<routecol.size(); i++)
    {
        for(int j=0; j<routecol[i].size(); j++)
            cout<<routecol[i][j]<<' ';
        cout<<endl;
    }
    cout<<"Total Distance: "<<total_distance<<endl;
}

void TabuRefinement(vector<vector<int>>& routecol, vector<vector<double>> distmatrix, vector<vector<double>> permitperiodcol, vector<double> visittime, vector<int> demand, int capacity)
{
    vector<int> total_demand;
    for(int i=0; i<routecol.size(); i++)
    {
        int total_demand_sub = 0;
        for(int j=0; j<routecol[i].size(); j++)
            total_demand_sub += demand[routecol[i][j]];
        total_demand.push_back(total_demand_sub);
    }

    int max_iter = 3;
    double diff_max = INT_MAX;
    vector<int> move;
    vector<int> prev_move; //tabu list
    for(int iter = 0; iter < max_iter; iter++)
    {
        diff_max = INT_MAX;
        move.clear();
        for(int i=0; i<routecol.size(); i++)
        {
            for(int j=1; j<routecol[i].size()-1; j++)
            {
                for(int k=0; k<routecol.size(); k++)
                {
                    if((total_demand[k]+demand[routecol[i][j]])>capacity)
                        continue;
                    if(k==i)
                        continue;
                    for(int l=1; l<routecol[k].size(); l++)
                    {
                        if(prev_move.size()>0)
                        {
                            if(i==prev_move[2]&&j==prev_move[3]&&k==prev_move[0]&&l==prev_move[1])
                                continue;
                        }
                        if(!checkArrivalTimesub(routecol[k], routecol[i][j], l, distmatrix, visittime, permitperiodcol))
                            continue;
                        double dist_diff;
                        dist_diff += (distmatrix[routecol[i][j-1]][routecol[i][j+1]]-(distmatrix[routecol[i][j]][routecol[i][j+1]]+distmatrix[routecol[i][j-1]][routecol[i][j]]));
                        dist_diff += ((distmatrix[routecol[k][l-1]][routecol[i][j]]+distmatrix[routecol[i][j]][routecol[k][l]])-distmatrix[routecol[k][l-1]][routecol[k][l]]);
                        if(dist_diff<diff_max)
                        {
                            diff_max = dist_diff;
                            if(move.size()==0)
                            {
                                move.push_back(i);
                                move.push_back(j);
                                move.push_back(k);
                                move.push_back(l);
                            }
                            else
                            {
                                move[0] = i;
                                move[1] = j;
                                move[2] = k;
                                move[3] = l;
                            }
                        }
                    }
                    }
                
            }
        }

        if(move.size()!=0)
        {
            total_demand[move[2]]+=demand[routecol[move[0]][move[1]]];
            total_demand[move[0]]-=demand[routecol[move[0]][move[1]]];
            routecol[move[2]].insert(routecol[move[2]].begin()+move[3], routecol[move[0]][move[1]]);
            routecol[move[0]].erase(routecol[move[0]].begin()+move[1]);
            prev_move.assign(move.begin(),move.end());
        }
    }

    if(!checkCapacity(routecol,demand,capacity))
        cout<<"The capacity of the car is surpassed"<<endl;
}

void route_initialize(vector<int>& route, vector<vector<double>> permitperiodcol, vector<bool>& visited)
{
    route.push_back(0);
    double earliest_value = INT_MAX-1;
    int earliest_id = 1;
    for(int i=1; i<permitperiodcol.size(); i++)
    {
        if(!visited[i])
        {
            if(permitperiodcol[i][1]<earliest_value)
            {
                earliest_value = permitperiodcol[i][1];
                earliest_id = i;
            }
        }
    }
    route.push_back(earliest_id);
    visited[earliest_id] = true;
    route.push_back(0);
}

vector<double> FindBestInsertion(vector<int> route, int insert_id, vector<vector<double>> distmatrix, vector<vector<double>> permitperiodcol, vector<double> visittime)
{
    //hyperparameters
    double alpha = 0.9;
    double mu = 0.1;
    double MIN_COST = INT_MAX-1;
    double mincost_id;

    vector<double> service_beginning_time;
    calInitialTiming(route, distmatrix, visittime, permitperiodcol, service_beginning_time);
    vector<double> bestinsersion;
    for(int i=1; i<route.size(); i++)
    {
        double dis_diff = distmatrix[route[i-1]][insert_id]+distmatrix[insert_id][route[i]]-mu*distmatrix[route[i-1]][route[i]];        
        double end_time;
        end_time = service_beginning_time[i-1]+visittime[route[i-1]]+distmatrix[route[i-1]][insert_id];
        if(end_time<permitperiodcol[insert_id][0])
            end_time = permitperiodcol[insert_id][0];
        if(end_time>permitperiodcol[insert_id][1])
            continue;
        end_time += (visittime[insert_id]+distmatrix[insert_id][route[i]]);
        if(end_time<permitperiodcol[route[i]][0])
            end_time = permitperiodcol[route[i]][0];
        if(end_time>permitperiodcol[route[i]][1])
            continue;
        if(!checkArrivalTimesub(route, insert_id, i, distmatrix, visittime, permitperiodcol))
            continue;
        
        double time_diff = end_time - service_beginning_time[i];
        double cost = alpha*dis_diff+(1-alpha)*time_diff;
        if(cost<MIN_COST)
        {
            MIN_COST = cost;
            mincost_id = i;
        }
    }

    if(MIN_COST!=INT_MAX-1)
    {
        bestinsersion.push_back(MIN_COST);
        bestinsersion.push_back(mincost_id);
    }

    return bestinsersion;    
}


void calInitialTiming(vector<int> route, vector<vector<double>> distmatrix, vector<double> visittime, vector<vector<double>> permitperiodcol, vector<double>& service_beginning_time)
{
    service_beginning_time.push_back(0);
    for(int i=1; i<route.size(); i++)
    {
        double start_service_time = service_beginning_time[i-1]+distmatrix[route[i-1]][route[i]]+visittime[route[i-1]];
        if(start_service_time<permitperiodcol[route[i]][0])
            start_service_time = permitperiodcol[route[i]][0];
        service_beginning_time.push_back(start_service_time);
    }
}

bool checkArrivalTime(vector<vector<int>> routecol, vector<vector<double>> distmatrix, vector<double> visittime, vector<vector<double>> permitperiodcol)
{
    double start_time;

    for(int i=0; i<routecol.size(); i++)
    {
        start_time = 0.0;
        bool flag = true;
        for(int j=1; j<routecol[i].size(); j++)
        {
            start_time += (distmatrix[routecol[i][j-1]][routecol[i][j]]+visittime[routecol[i][j-1]]);
            if(start_time<=permitperiodcol[routecol[i][j]][0])
                start_time = permitperiodcol[routecol[i][j]][0];
            if(start_time>permitperiodcol[routecol[i][j]][1])
            {
                cout<<i<<' '<<j<<' '<<start_time<<' '<<permitperiodcol[routecol[i][j]][1]<<endl;
                flag = false;
                break;
            }
        }

        if(!flag)
            return false;
    }

    return true;    
}

bool checkArrivalTimesub(vector<int> route, int insert_obj, int insert_place, vector<vector<double>> distmatrix, vector<double> visittime, vector<vector<double>> permitperiodcol)
{
    double start_time = 0.0;
    for(int i=1; i<route.size(); i++)
    {
        start_time += visittime[route[i-1]];
        if(i==insert_place)
        {
            start_time += distmatrix[route[i-1]][insert_obj];
            if(start_time<permitperiodcol[insert_obj][0])
                start_time = permitperiodcol[insert_obj][0];
            if(start_time>permitperiodcol[insert_obj][1])
                return false;
            start_time += (visittime[insert_obj]+distmatrix[insert_obj][route[i]]);
        }
        else
            start_time += distmatrix[route[i-1]][route[i]];
        
        if(start_time<permitperiodcol[route[i]][0])
            start_time = permitperiodcol[route[i]][0];
        if(start_time>permitperiodcol[route[i]][1])
            return false;        
    }

    return true;
}

bool checkCapacity(vector<vector<int>> routecol, vector<int> demand, int capacity)
{
    int total_demand;
    for(int i=0; i<routecol.size(); i++)
    {
        total_demand = 0;
        for(int j=0; j<routecol[i].size(); j++)
            total_demand += demand[routecol[i][j]];
        if(total_demand>capacity)
            return false;
    }
    return true;
}


double calTotalDistance(vector<vector<int>> routecol, vector<vector<double>> distmatrix)
{
    double total_dist = 0.0;

    for(int i=0; i<routecol.size(); i++)
    {
        for(int j=1; j<routecol[i].size(); j++)
            total_dist+=distmatrix[routecol[i][j-1]][routecol[i][j]];
    }

    return total_dist;
}




