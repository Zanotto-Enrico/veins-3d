#include "veins/modules/utility/PolygonTriangulation.h"
#include <utility>
#include <math.h>
#include <algorithm>
#include <list>
#include <map>
#include <deque>

using veins::Coord;
using veins::Edge;
using veins::vertexType;
using veins::Triangle;

// Calculate arc length between two indices in a cyclic array
int veins::arc_length(int i, int j, int N) {
    return std::min(std::abs(i - j), N - std::abs(i - j));
}


// finds the first edge over a given point
const Edge* veins::findUpperBound(double y,double x, std::set<Edge> &bounds)
{
    std::set<Edge>::iterator it = bounds.begin();    // can be improved
    while (it != bounds.end()) {
        if (y < it->start.y + ((it->end.y - it->start.y) / (it->end.x - it->start.x)) * (x - it->start.x)) {
            return &(*it);
        }
        ++it;
    }

    return nullptr;  
}

bool veins::isCounterClockwise(const std::vector<Coord>& vertices) {
    double sum = 0.0;
    size_t numVertices = vertices.size();

    for (size_t i = 0; i < numVertices; ++i) {
        const Coord& current = vertices[i];
        const Coord& next = vertices[(i + 1) % numVertices]; 
        sum += (next.x - current.x) * (next.y + current.y);
    }

    return sum > 0.0; // If the sum id positive the perimeter is CCW
}

vertexType veins::getVertexType(const Coord &vertex, const Coord &next, const Coord &prev )
{
    if(prev.x == vertex.x && vertex.x == next.x && prev.y > next.y)
        return REGULAR_UPPER;
    if(prev.x == vertex.x && vertex.x == next.x && prev.y < next.y)
        return REGULAR_LOWER;
    if(prev.x <= vertex.x && next.x < vertex.x )
    {
        Coord vector_pc = Coord(vertex.x - prev.x, vertex.y - prev.y, 0.0, 0);
        Coord vector_pn = Coord(next.x - prev.x, next.y - prev.y, 0.0, 0);
        double cross_product = vector_pc.x * vector_pn.y - vector_pc.y * vector_pn.x; // cross product between 2 vectors
        
        if(cross_product < 0)   return MERGE;
        else                    return END;
    }
    
    if(prev.x >= vertex.x && next.x > vertex.x )
    {
        Coord vector_pc = Coord(vertex.x - prev.x, vertex.y - prev.y, 0.0, 0);
        Coord vector_pn = Coord(next.x - prev.x, next.y - prev.y, 0.0, 0);
        double cross_product = vector_pc.x * vector_pn.y - vector_pc.y * vector_pn.x; // cross product between 2 vectors
        
        if(cross_product < 0)   return SPLIT;
        else                    return START;
    }
    if( next.x <= vertex.x && vertex.x <= prev.x)   return REGULAR_UPPER;
    if( next.x >= vertex.x && vertex.x >= prev.x)   return REGULAR_LOWER;
}

/*
    sweep line algorithm to partition non monoton polygons in monotone polygons
*/
std::vector<std::vector<Coord>> veins::partitionPolygonIntoMonotone(std::vector<Coord>& polygon)
{
    bool isCCW = isCounterClockwise(polygon);
    for (size_t i = 0; i < polygon.size(); ++i)
    {
        if(isCCW) polygon[i].y = -polygon[i].y;
        polygon[i].index = i;
        //temp fix
        if(polygon[i].x == polygon[(i+1)%polygon.size()].x) polygon[i].x = polygon[i].x+0.000001;
    }
    int vertices_num = polygon.size();
    // Create the event queue and initialize it with vertex and edge events.
    std::vector<Coord> eventQueue = polygon;
    std::sort(eventQueue.begin(), eventQueue.end());

    // Sweep line partitions boundaries
    std::set<Edge> activeEdges;

    // vecotor of all monotone polygons found divided in upper and lower chains
    std::vector<std::pair<std::vector<Coord>,std::vector<Coord>>> monotones;

    for (const Coord& event : eventQueue)
    {

        Coord prev = polygon[(event.index+vertices_num-1)%vertices_num];
        Coord next = polygon[(event.index+1)%vertices_num];

        // Determine whether this is a merge vertex, split vertex, or regular vertex.
        vertexType type = getVertexType(event, next, prev);

        if (type == MERGE) 
        {
            const Edge* e = findUpperBound(event.y,event.x, activeEdges);

            // if the last vertex seen in the upper polygon is a merge-vertex
            if(e->getMergeMonotonePolygonIndex() != -1)
                monotones[e->getMergeMonotonePolygonIndex()].first.push_back(event);

            const Edge* e2 = &*activeEdges.find(Edge(event,next));

            // if the last vertex seen in the lower polygon is a merge-vertex
            if(e2->getMergeMonotonePolygonIndex() != -1)
            {
                e->setMergeMonotonePolygonIndex(e2->getMergeMonotonePolygonIndex());
                monotones[e2->getMonotonePolygonIndex()].first.push_back(event);
            }
            else
                e->setMergeMonotonePolygonIndex(e2->getMonotonePolygonIndex());

            e->setHelper(event);
            monotones[e->getMonotonePolygonIndex()].second.push_back(event);
            monotones[e->getMergeMonotonePolygonIndex()].first.push_back(event);
            activeEdges.erase(Edge(event,next));
        }
        else if (type == SPLIT) 
        {
            const Edge* e = findUpperBound(event.y,event.x, activeEdges);
            if(e->getMergeMonotonePolygonIndex() == -1)
            {
                monotones.push_back(std::make_pair(std::vector<Coord>{*e->getHelper()}, std::vector<Coord>{}));
                if(*e->getHelper() == e->start) // if the helper is in the upper-chain
                {
                    activeEdges.insert(Edge(prev,event,event,e->getMonotonePolygonIndex()));
                    monotones[e->getMonotonePolygonIndex()].first.push_back(event);
                    e->setMonotonePolygonIndex( monotones.size()-1);
                    monotones[ e->getMonotonePolygonIndex()].second.push_back(event);
                }
                else                            // if the helper is in the lower-chain
                {
                    activeEdges.insert(Edge(prev,event,event,monotones.size()-1));
                    monotones[e->getMonotonePolygonIndex()].second.push_back(event);
                    monotones[monotones.size()-1].first.push_back(event);
                }
            }
            else                                // if the helper is a previous merge-vertex
            {
                activeEdges.insert(Edge(prev,event,event, e->getMergeMonotonePolygonIndex()));
                monotones[e->getMergeMonotonePolygonIndex()].first.push_back(event);
                monotones[e->getMonotonePolygonIndex()].second.push_back(event);
            }
            
            
            e->setHelper(event);
            e->setMergeMonotonePolygonIndex(-1);
        }
        else if(type == START) 
        {
            monotones.push_back(std::make_pair(std::vector<Coord>{event}, std::vector<Coord>{}));
            activeEdges.insert(Edge(prev,event,event,monotones.size()-1));
        }
        else if(type == END) 
        {
            auto near = activeEdges.find(Edge(event,next));
            if(near->getMergeMonotonePolygonIndex() != -1)
            {
                monotones[near->getMergeMonotonePolygonIndex()].second.push_back(event);
                near->setMergeMonotonePolygonIndex(-1);
            }
            monotones[near->getMonotonePolygonIndex()].second.push_back(event);
            activeEdges.erase(Edge(event,next)); 
        }
        else if (type == REGULAR_UPPER) 
        {
            const Edge* e = &(*activeEdges.find(Edge(event,next)));
            if(e->getMergeMonotonePolygonIndex() != -1)
            {
                monotones[e->getMergeMonotonePolygonIndex()].first.push_back(event);
                activeEdges.insert(Edge(event,prev,event,e->getMergeMonotonePolygonIndex()));
                e->setMergeMonotonePolygonIndex(-1);
            }
            else
                activeEdges.insert(Edge(event,prev,event,e->getMonotonePolygonIndex()));
            monotones[e->getMonotonePolygonIndex()].first.push_back(event);
            activeEdges.erase(Edge(event,next));
        }
        else if (type == REGULAR_LOWER) 
        {
            const Edge* e = findUpperBound(event.y,event.x, activeEdges);
            if(e->getMergeMonotonePolygonIndex() != -1)
            {
                monotones[e->getMergeMonotonePolygonIndex()].second.push_back(event);
                e->setMergeMonotonePolygonIndex(-1);
            }
            e->setHelper(event);
            monotones[e->getMonotonePolygonIndex()].second.push_back(event);
        }
    }

    std::vector<std::vector<Coord>> monotonePolygons;
    for (auto m : monotones)
    {
        std::vector<Coord> newPol;
        for (int i = m.first.size() - 1; i >= 0; --i)
        {
            if(isCCW) m.first[i].y = -m.first[i].y;
            newPol.push_back(m.first[i]);
        }
        for (int i = 0 ; i < m.second.size(); ++i) 
        {
            if(isCCW) m.second[i].y = -m.second[i].y;
            newPol.push_back(m.second[i]);
        }
        monotonePolygons.push_back(newPol);
    }
    return monotonePolygons;
}

int veins::DetermineConvexity(const Coord& p1, const Coord& p2, const Coord& p3)
{
    Coord v1 = {p2.x - p1.x, p2.y - p1.y, p2.z - p1.z};
    Coord v2 = {p3.x - p1.x, p3.y - p1.y, p3.z - p1.z};

    Coord cross = v1.crossProduct(v2);
    
    if (cross.x > 0 || (cross.x == 0 && (cross.y > 0 || (cross.y == 0 && cross.z > 0))))
        return 1; // Curva concava
    else if (cross.x < 0 || (cross.x == 0 && (cross.y < 0 || (cross.y == 0 && cross.z < 0))))
        return -1; // Curva convessa
    else
        return 0; // Punti allineati
}

std::vector<Triangle> veins::triangulateMonotonePolygon(const std::vector<Coord>& polygon) {
    std::vector<Triangle> triangles;
    int poly_len = polygon.size();

    if(poly_len < 3 ) return  std::vector<Triangle>();

    std::vector<Coord> eventQueue = polygon;
    for (size_t i = 0; i < polygon.size(); ++i)
        eventQueue[i].index = i;
    std::sort(eventQueue.begin(), eventQueue.end());
    std::deque<Coord> deque;
    deque.push_front(eventQueue[0]);
    deque.push_front(eventQueue[1]);

    bool isCCW = isCounterClockwise(polygon);
    for (int i = 2; i < poly_len; ++i)
    {
        Coord current = eventQueue[i];
        Coord prev = polygon[(current.index+poly_len-1)%poly_len];
        Coord next = polygon[(current.index+1)%poly_len];

        Coord last = deque.front();
        vertexType lastType = getVertexType(last, polygon[(last.index+1)%poly_len], polygon[(last.index+poly_len-1)%poly_len]);

        vertexType type = getVertexType(current,next,prev);
        if(type == lastType)
        {
            deque.pop_front();
            int convexity = DetermineConvexity(current,last,deque.front());
            while ( (( convexity < 0 && next.x < current.x && current.x < prev.x  && isCCW) || 
                     ( convexity > 0 && next.x > current.x && current.x > prev.x  && isCCW) ||
                     ( convexity < 0 && next.x > current.x && current.x > prev.x  && !isCCW) || 
                     ( convexity > 0 && next.x < current.x && current.x < prev.x  && !isCCW)) && 
                    deque.size() > 0)            
            {
                triangles.push_back({deque.front(),last,current});
                last = deque.front();
                deque.pop_front();
                convexity = DetermineConvexity(current,last,deque.front());
            }
            deque.push_front(last);
            deque.push_front(current);
        }
        else
        {
            Coord tmp = deque.back();
            deque.pop_back();
            while(deque.size() >= 2)
            {
                triangles.push_back({tmp,current,deque.back()});
                tmp = deque.back();          
                deque.pop_back();                
            }
            triangles.push_back({tmp,current,deque.back()});
            deque.push_front(current);
        }   
    }

    return triangles;
}
