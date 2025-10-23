#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <cmath>
#include <tinyxml2.h>
#include <queue>
#include <sstream>
#include <algorithm>

using namespace tinyxml2;

const int GRID_SIZE = 200;

struct Cell { bool blocked=false; bool isStart=false; bool isEnd=false; };
struct Node { int x,y; double g=0,h=0; Node* parent=nullptr; double f() const { return g+h; } };
struct CompareNode { bool operator()(Node* a, Node* b){ return a->f() > b->f(); } };
const int dx[8] = {-1,1,0,0,-1,-1,1,1};
const int dy[8] = {0,0,-1,1,-1,1,-1,1};
const double moveCost[8] = {1,1,1,1,1.414,1.414,1.414,1.414};

double heuristic(int x1,int y1,int x2,int y2){ return std::sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)); }
int scaleToGrid(double coord,double svgMax){ return std::round(coord/(svgMax)*(GRID_SIZE-1)); }
bool pointInCircle(int x,int y,double cx,double cy,double r){ double dx=x-cx, dy=y-cy; return dx*dx+dy*dy<=r*r; }
bool pointOnLine(int x,int y,double x1,double y1,double x2,double y2,double thickness){
    double A=x-x1, B=y-y1, C=x2-x1, D=y2-y1;
    double dot=A*C + B*D, len_sq=C*C+D*D;
    double param = len_sq!=0 ? dot/len_sq : -1;
    double xx, yy;
    if(param<0){ xx=x1; yy=y1; }
    else if(param>1){ xx=x2; yy=y2; }
    else{ xx=x1+param*C; yy=y1+param*D; }
    double dx=x-xx, dy=y-yy;
    return dx*dx+dy*dy<=thickness*thickness;
}

std::vector<Node> astar(Cell grid[GRID_SIZE][GRID_SIZE], int sx,int sy,int ex,int ey){
    bool closed[GRID_SIZE][GRID_SIZE] = {};
    Node* allNodes[GRID_SIZE][GRID_SIZE] = {};
    std::priority_queue<Node*, std::vector<Node*>, CompareNode> open;

    Node* start = new Node{sx,sy};
    start->h = heuristic(sx,sy,ex,ey);
    allNodes[sx][sy]=start;
    open.push(start);

    while(!open.empty()){
        Node* current = open.top(); open.pop();
        if(current->x==ex && current->y==ey){
            std::vector<Node> path;
            while(current){ path.push_back(*current); current=current->parent; }
            std::reverse(path.begin(), path.end());
            return path;
        }
        closed[current->x][current->y]=true;
        for(int dir=0;dir<8;dir++){
            int nx=current->x+dx[dir], ny=current->y+dy[dir];
            if(nx<0||ny<0||nx>=GRID_SIZE||ny>=GRID_SIZE) continue;
            if(grid[nx][ny].blocked||closed[nx][ny]) continue;
            double gNew=current->g+moveCost[dir];
            if(allNodes[nx][ny]==nullptr){
                Node* neighbor=new Node{nx,ny};
                neighbor->g=gNew;
                neighbor->h=heuristic(nx,ny,ex,ey);
                neighbor->parent=current;
                allNodes[nx][ny]=neighbor;
                open.push(neighbor);
            } else if(gNew<allNodes[nx][ny]->g){
                allNodes[nx][ny]->g=gNew;
                allNodes[nx][ny]->parent=current;
                open.push(allNodes[nx][ny]);
            }
        }
    }
    return {};
}

int main(){
    Cell grid[GRID_SIZE][GRID_SIZE];
    XMLDocument doc;
    if(doc.LoadFile("map.svg")!=XML_SUCCESS){ std::cout<<"Failed to load SVG\n"; return 1; }
    XMLElement* svg=doc.FirstChildElement("svg");
    if(!svg){ std::cout<<"No <svg> element found\n"; return 1; }

    double svgWidth=4000, svgHeight=4000;

    std::vector<std::tuple<double,double,double,double>> walls;
    for(XMLElement* path=svg->FirstChildElement("g")->FirstChildElement("path"); path; path=path->NextSiblingElement("path")){
        const char* d=path->Attribute("d");
        if(!d) continue;
        std::stringstream ss(d);
        char cmd; double x1,y1,x2,y2;
        ss>>cmd;
        if(cmd=='M'||cmd=='m'){
            ss>>x1; ss.ignore(); ss>>y1;
            if(ss>>x2){ ss.ignore(); ss>>y2; } else { x2=x1; y2=y1; }
            walls.emplace_back(x1,y1,x2,y2);
        }
    }

    double ax=0,ay=0,ar=0,bx=0,by=0,br=0;
    for(XMLElement* circ=svg->FirstChildElement("g")->FirstChildElement("circle"); circ; circ=circ->NextSiblingElement("circle")){
        double cx=circ->DoubleAttribute("cx"), cy=circ->DoubleAttribute("cy"), r=circ->DoubleAttribute("r");
        const char* id=circ->Attribute("id");
        if(id && std::string(id)=="A"){ ax=cx; ay=cy; ar=r; }
        else if(id && std::string(id)=="B"){ bx=cx; by=cy; br=r; }
    }
    for(XMLElement* ell=svg->FirstChildElement("g")->FirstChildElement("ellipse"); ell; ell=ell->NextSiblingElement("ellipse")){
        double cx=ell->DoubleAttribute("cx"), cy=ell->DoubleAttribute("cy");
        double rx=ell->DoubleAttribute("rx"), ry=ell->DoubleAttribute("ry");
        const char* id=ell->Attribute("id");
        if(id && std::string(id)=="B"){ bx=cx; by=cy; br=(rx+ry)/2; }
    }

    double thickness=20.0;
    for(int i=0;i<GRID_SIZE;i++){
        for(int j=0;j<GRID_SIZE;j++){
            double x=i*(svgWidth/GRID_SIZE), y=j*(svgHeight/GRID_SIZE);
            for(auto &w: walls){
                double x1,y1,x2,y2; std::tie(x1,y1,x2,y2)=w;
                if(pointOnLine(x,y,x1,y1,x2,y2,thickness)){ grid[i][j].blocked=true; break; }
            }
            if(pointInCircle(x,y,ax,ay,ar)) grid[i][j].isStart=true;
            if(pointInCircle(x,y,bx,by,br)) grid[i][j].isEnd=true;
        }
    }

    int sx=-1,sy=-1,ex=-1,ey=-1;
    for(int i=0;i<GRID_SIZE;i++) for(int j=0;j<GRID_SIZE;j++){
        if(grid[i][j].isStart){ sx=i; sy=j; }
        if(grid[i][j].isEnd){ ex=i; ey=j; }
    }
    if(sx==-1||ex==-1){ std::cout<<"Start or End not found!\n"; return 1; }

    auto path=astar(grid,sx,sy,ex,ey);
    if(path.empty()){ std::cout<<"No path found!\n"; return 1; }

    // --- Output new SVG ---
    std::ofstream out("map_with_path.svg");
    out << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n";
    out << "<svg width=\"200cm\" height=\"200cm\" viewBox=\"0 0 4000 4000\" xmlns=\"http://www.w3.org/2000/svg\">\n";

    // Original walls
    for(auto &w: walls){
        double x1,y1,x2,y2; std::tie(x1,y1,x2,y2)=w;
        out << "<line x1=\"" << x1 << "\" y1=\"" << y1 << "\" x2=\"" << x2 << "\" y2=\"" << y2
            << "\" stroke=\"red\" stroke-width=\"20\" />\n";
    }

    // Original circles
    out << "<circle cx=\"" << ax << "\" cy=\"" << ay << "\" r=\"" << ar
        << "\" fill=\"red\" />\n";
    out << "<ellipse cx=\"" << bx << "\" cy=\"" << by << "\" rx=\"" << br << "\" ry=\"" << br
        << "\" fill=\"red\" />\n";

    // Draw path
    out << "<polyline fill=\"none\" stroke=\"blue\" stroke-width=\"15\" points=\"";
    for(auto &n:path){
        double x=n.x*(svgWidth/GRID_SIZE);
        double y=n.y*(svgHeight/GRID_SIZE);
        out << x << "," << y << " ";
    }
    out << "\" />\n";

    out << "</svg>\n";
    out.close();

    std::cout<<"SVG with path saved as map_with_path.svg\n";
    return 0;
}
