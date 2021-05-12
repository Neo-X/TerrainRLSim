// Geometric Tools LLC, Redmond WA 98052
// Copyright (c) 1998-2015
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 2.3.0 (2016/03/07)

#pragma once

#include <LowLevel/GteLogger.h>
#include <LowLevel/GteMinHeap.h>
#include <array>
#include <map>
#include <memory>
#include <set>
#include <stack>

// Extract the minimal cycle basis for a planar graph.  The input vertices and
// edges must form a graph for which edges intersect only at vertices; that is,
// no two edges must intersect at an interior point of one of the edges.  The
// algorithm is described in 
//   http://www.geometrictools.com/Documentation/MinimalCycleBasis.pdf
// The graph might have filaments, which are polylines in the graph that are
// not shared by a cycle.  These are also extracted by the implementation.
// Because the inputs to the constructor are vertices and edges of the graph,
// isolated vertices are ignored.
//
// The computations that determine which adjacent vertex to visit next during
// a filament or cycle traversal do not require division, so the exact
// arithmetic type BSNumber<UIntegerAP32> suffices for ComputeType when you
// want to ensure a correct output.  (Floating-point rounding errors
// potentially can lead to an incorrect output.)

namespace gte
{

template <typename Real>
class MinimalCycleBasis
{
public:
    // The cycle basis is stored within a forest of trees of primitives
    // (simple cycles and filaments), one tree per simply connected graph
    // component.  The forest shows the geometric/ ordering and nesting of
    // the primitives.
    struct PrimitiveTree
    {
        std::vector<int> cycle;
        std::vector<std::vector<int>> filaments;
        std::vector<PrimitiveTree> children;
    };

    // The input positions and edges must form a planar graph for which edges
    // intersect only at vertices; that is, no two edges must intersect at an
    // interior point of one of the edges.
    MinimalCycleBasis(
        std::vector<std::array<Real, 2>> const& positions,
        std::vector<std::array<int, 2>> const& edges,
        std::vector<PrimitiveTree>& forest);

private:
    // No copy or assignment allowed.
    MinimalCycleBasis(MinimalCycleBasis const&) = delete;
    MinimalCycleBasis& operator=(MinimalCycleBasis const&) = delete;

    struct Vertex
    {
        Vertex(int inName, std::array<Real, 2> const* inPosition);

        // The index into the 'positions' input provided to the call to
        // operator().  The index is used when reporting cycles to the
        // caller of the constructor for MinimalCycleBasis.
        int name;

        // Multiple vertices can share a position during processing of
        // graph components.
        std::array<Real, 2> const* position;

        // The std::vector<std::shared_ptr<Vertex>> arrays keep the
        // reference counts on the Vertex objects.  The adjacent pointers
        // are considered to be weak pointers (neither object ownership nor
        // reference counting is required).
        std::set<Vertex*> adjacent;

        // Support for depth-first traversal of a graph.
        int visited;

        bool operator< (Vertex const& vertex) const
        {
            return name < vertex.name;
        }
    };

    struct HeapElement
    {
        // Filaments have at least one endpoint with 'numAdjacencies' of 1.
        // All other vertices have 'numAdjacencies' 2 or larger.  The
        // algorithm removes the filaments first.  Once the heap is updated
        // with these removed, the remaining vertices are ordered using
        // lexigographical sorting.  This allows us to find the starting
        // vertex for traversal of a subgraph.
        int numAdjacencies;
        std::array<Real, 2> const* position;

        bool operator== (HeapElement const& element) const;
        bool operator< (HeapElement const& element) const;
        bool operator<= (HeapElement const& element) const;
    };
    typedef MinHeap<Vertex*, HeapElement> Heap;
    typedef typename MinHeap<Vertex*, HeapElement>::Record Record;
    typedef std::map<Vertex*, Record*> RecordMap;

    struct HeapManager
    {
        HeapManager(int numElements);
        void Insert(Vertex* vertex);
        bool Update(Vertex* vertex);
        bool Remove(Vertex* vertex);

        Heap heap;
        RecordMap records;
    };

    // The constructor uses GetComponents(...) and DepthFirstSearch(...) to
    // get the connected components of the graph implied by the input 'edges'.
    // Recursive processing uses only DepthFirstSearch(...) to collect
    // vertices of the subgraphs of the original graph.
    static void DepthFirstSearch(Vertex* vInitial, std::vector<Vertex*>& component);

    // Support for traversing a simply connected component of the graph.
    bool ExtractPrimitives(std::vector<Vertex*>& component, PrimitiveTree& tree);

    bool ExtractFilament(Vertex* vertex, HeapManager& manager, PrimitiveTree& tree);

    bool ExtractCycles(Vertex* vertex, HeapManager& manager, PrimitiveTree& tree);

    bool Process(std::vector<Vertex*>& closedWalk, HeapManager& manager, PrimitiveTree& tree);
    void ExtractCycle(std::vector<Vertex*>& closedWalk, HeapManager& manager, PrimitiveTree& tree);

    Vertex* GetClockwiseMost(Vertex* vPrev, Vertex* vCurr) const;
    Vertex* GetCounterclockwiseMost(Vertex* vPrev, Vertex* vCurr) const;

    // Storage for referenced vertices of the original graph and for new
    // vertices added during graph traversal.
    std::vector<std::shared_ptr<Vertex>> mVertexStorage;
};


template <typename Real>
MinimalCycleBasis<Real>::MinimalCycleBasis(
    std::vector<std::array<Real, 2>> const& positions,
    std::vector<std::array<int, 2>> const& edges,
    std::vector<PrimitiveTree>& forest)
{
    forest.clear();
    if (positions.size() == 0 || edges.size() == 0)
    {
        // The graph is empty, so there are no filaments or cycles.
        return;
    }

    // Determine the unique positions referenced by the edges.
    std::map<int, std::shared_ptr<Vertex>> unique;
    for (auto const& edge : edges)
    {
        for (int i = 0; i < 2; ++i)
        {
            int name = edge[i];
            if (unique.find(name) == unique.end())
            {
                std::shared_ptr<Vertex> vertex =
                    std::make_shared<Vertex>(name, &positions[name]);
                unique.insert(std::make_pair(name, vertex));

            }
        }
    }

    // Assign responsibility for ownership of the Vertex objects.
    std::vector<Vertex*> vertices;
    mVertexStorage.reserve(unique.size());
    vertices.reserve(unique.size());
    for (auto const& element : unique)
    {
        mVertexStorage.push_back(element.second);
        vertices.push_back(element.second.get());
    }

    // Determine the adjacencies from the edge information.
    for (auto const& edge : edges)
    {
        auto iter0 = unique.find(edge[0]);
        auto iter1 = unique.find(edge[1]);
        iter0->second->adjacent.insert(iter1->second.get());
        iter1->second->adjacent.insert(iter0->second.get());
    }

    // Get the connected components of the graph.  The 'visited' flags are
    // 0 (unvisited), 1 (discovered), 2 (finished).  The Vertex constructor
    // sets all 'visited' flags to 0.
    std::vector<std::vector<Vertex*>> components;
    for (auto vInitial : mVertexStorage)
    {
        if (vInitial->visited == 0)
        {
            components.push_back(std::vector<Vertex*>());
            DepthFirstSearch(vInitial.get(), components.back());
        }
    }

    // The depth-first search is used later for collecting vertices for
    // subgraphs that are detached from the main graph, so the 'visited'
    // flags must be reset to zero after component finding.
    for (auto vertex : mVertexStorage)
    {
        vertex->visited = 0;
    }

    // Get the primitives for the components.
    int debugCounter = 0;
    for (auto& component : components)
    {
        PrimitiveTree tree;
        if (ExtractPrimitives(component, tree))
        {
            forest.push_back(tree);
            ++debugCounter;
        }
    }
}

template <typename Real>
void MinimalCycleBasis<Real>::DepthFirstSearch(Vertex* vInitial, std::vector<Vertex*>& component)
{
    std::stack<Vertex*> vStack;
    vStack.push(vInitial);
    while (vStack.size() > 0)
    {
        Vertex* vertex = vStack.top();
        vertex->visited = 1;
        size_t i = 0;
        for (auto adjacent : vertex->adjacent)
        {
            if (adjacent && adjacent->visited == 0)
            {
                vStack.push(adjacent);
                break;
            }
            ++i;
        }

        if (i == vertex->adjacent.size())
        {
            vertex->visited = 2;
            component.push_back(vertex);
            vStack.pop();
        }
    }
}

template <typename Real>
bool MinimalCycleBasis<Real>::ExtractPrimitives(std::vector<Vertex*>& component, PrimitiveTree& tree)
{
    // Build the priority queue for the vertices, sorted first by number of
    // adjacent neighbors and second by lexicographical order of position.
    HeapManager manager(static_cast<int>(component.size()));
    for (auto vertex : component)
    {
        manager.Insert(vertex);
    }

    // Extract the cycles and filaments.
    while (manager.heap.GetNumElements() > 0)
    {
        Vertex* vertex = nullptr;
        HeapElement element;
        manager.heap.GetMinimum(vertex, element);
        if (element.numAdjacencies == 1)
        {
            if (!ExtractFilament(vertex, manager, tree))
            {
                return false;
            }
        }
        else if (element.numAdjacencies > 1)
        {
            PrimitiveTree childTree;
            if (!ExtractCycles(vertex, manager, childTree))
            {
                return false;
            }
            tree.children.push_back(childTree);
        }
        else
        {
            LogError("Unexpected condition.");
            return false;
        }
    }

    return true;
}

template <typename Real>
bool MinimalCycleBasis<Real>::ExtractFilament(Vertex* vertex, HeapManager& manager,
    PrimitiveTree& tree)
{
    if (!vertex || vertex->adjacent.size() != 1)
    {
        LogError("Unexpected condition.");
        return false;
    }

    // Traverse the filament.
    std::vector<Vertex*> ptrFilament;
    while (vertex->adjacent.size() == 1)
    {
        // Add the current vertex to the filament.
        ptrFilament.push_back(vertex);

        // Remove the current vertex from the heap.
        if (!manager.Remove(vertex))
        {
            LogError("Unexpected condition.");
            return false;
        }

        // Get the sole adjacent vertex and update it.
        Vertex* adjacent = *vertex->adjacent.begin();
        if (!adjacent)
        {
            LogError("Unexpected condition.");
            return false;
        }
        adjacent->adjacent.erase(vertex);
        if (!manager.Update(adjacent))
        {
            LogError("Unexpected condition.");
            return false;
        }

        // Traverse to the adjacent vertex.
        vertex = adjacent;
    }

    // The number of adjacent vertices to v is either 0 (filament is a
    // connected component of the graph, an open polyline) or larger than 1
    // (filament endpoint was a branch point of the graph before removing
    // the final edge).  In either case, v is the endpoint of the filament.
    ptrFilament.push_back(vertex);
    if (vertex->adjacent.size() == 0)
    {
        // The filament is a connected component.  Remove the other endpoint
        // from the heap.
        HeapElement element;
        manager.heap.Remove(vertex, element);
    }

    // Report the filament in terms of the original position indices.
    std::vector<int> filament(ptrFilament.size());
    for (size_t i = 0; i < filament.size(); ++i)
    {
        filament[i] = ptrFilament[i]->name;
    }
    tree.filaments.push_back(filament);
    return true;
}

template <typename Real>
bool MinimalCycleBasis<Real>::ExtractCycles(Vertex* vertex, HeapManager& manager,
    PrimitiveTree& tree)
{
    if (!vertex || vertex->adjacent.size() <= 1)
    {
        LogError("Unexpected condition.");
        return false;
    }

    // Traverse the closed walk, duplicating the starting vertex as the
    // last vertex.
    std::vector<Vertex*> closedWalk;
    Vertex* vStart = vertex;
    closedWalk.push_back(vStart);
    Vertex* vAdj = GetClockwiseMost(nullptr, vStart);
    while (vAdj != vStart)
    {
        closedWalk.push_back(vAdj);
        Vertex* vNext = GetCounterclockwiseMost(vertex, vAdj);
        vertex = vAdj;
        vAdj = vNext;
    }
    closedWalk.push_back(vStart);

    // Recursively process the closed walk to extract simple cycles.
    return Process(closedWalk, manager, tree);
}

template <typename Real>
bool MinimalCycleBasis<Real>::Process(std::vector<Vertex*>& closedWalk,
    HeapManager& manager, PrimitiveTree& tree)
{
    std::map<Vertex*, int> duplicates;
    int numClosedWalk = static_cast<int>(closedWalk.size());
    for (int i = 1; i + 1 < numClosedWalk; ++i)
    {
        auto diter = duplicates.find(closedWalk[i]);
        if (diter != duplicates.end())
        {
            // The vertex has been visited previously, so we have a smaller
            // closed walk to visit.  The resulting envelope of the walk
            // contains a subgraph that connects to the original graph at a
            // single vertex.
            int iStart = diter->second, iFinal = i;
            if (iFinal - iStart > 2)
            {
                // Encountered a repeated vertex in the closed walk.
                Vertex* original = diter->first;
                std::shared_ptr<Vertex> clone =
                    std::make_shared<Vertex>(original->name, original->position);
                mVertexStorage.push_back(clone);

                // The clone will manage the adjacents for the original that lie
                // inside the wedge defined by the first and last edges of the
                // closed walk.
                Vertex* minVertex = closedWalk[iStart + 1];
                Vertex* maxVertex = closedWalk[iFinal - 1];

                original->adjacent.erase(minVertex);
                minVertex->adjacent.erase(original);
                clone->adjacent.insert(minVertex);
                minVertex->adjacent.insert(clone.get());

                original->adjacent.erase(maxVertex);
                maxVertex->adjacent.erase(original);
                clone->adjacent.insert(maxVertex);
                maxVertex->adjacent.insert(clone.get());

                std::array<Real, 2> dMin, dMax;
                for (int j = 0; j < 2; ++j)
                {
                    dMin[j] = (*minVertex->position)[j] - (*original->position)[j];
                    dMax[j] = (*maxVertex->position)[j] - (*original->position)[j];
                }

                std::set<Vertex*> adjacent = original->adjacent;
                for (auto vertex : adjacent)
                {
                    std::array<Real, 2> dVer;
                    for (int j = 0; j < 2; ++j)
                    {
                        dVer[j] = (*vertex->position)[j] - (*original->position)[j];
                    }

                    if (dVer[0] * dMin[1] - dVer[1] * dMin[0] >(Real)0
                        && dVer[0] * dMax[1] - dVer[1] * dMax[0] < (Real)0)
                    {
                        original->adjacent.erase(vertex);
                        vertex->adjacent.erase(original);
                        clone->adjacent.insert(vertex);
                        vertex->adjacent.insert(clone.get());
                    }
                }

                // We have erased at least two adjacents from the original vertex,
                // so the heap must be updated accordingly.
                manager.Update(original);

                // Get the (single) component representing the subgraph bounded
                // by the smaller closed walk.
                std::vector<Vertex*> component;
                DepthFirstSearch(clone.get(), component);

                // Extract the primitives from the subgraph.
                if (!ExtractPrimitives(component, tree))
                {
                    return false;
                }

                // The vertices of the primitives were removed from subheaps
                // during the ExtractPrimitives(...) call.  Now remove them from
                // the caller heap.
                for (auto vertex : component)
                {
                    manager.Remove(vertex);
                }

                // Remove the envelope of the smaller closed walk from the
                // input closed walk.
                closedWalk.erase(closedWalk.begin() + iStart + 1, closedWalk.begin() + iFinal + 1);
                numClosedWalk = static_cast<int>(closedWalk.size());
                if (numClosedWalk == 3)
                {
                    // The remaining closed walk is a single disconnected edge.
                    return ExtractFilament(closedWalk[1], manager, tree);
                }
                i = iStart;
            }
            else
            {
                // After removing a cycle, a filament exists and must be
                // removed.
                int endIndex = iStart + 1;  // endIndex <= (closedWalk.size() - 1)/2
                if (!ExtractFilament(closedWalk[endIndex], manager, tree))
                {
                    return false;
                }

                auto const& filament = tree.filaments.back();
                int const numVertices = static_cast<int>(filament.size());
                if (endIndex + 1 > numVertices)
                {
                    // The filament is strictly contained by the closed walk.
                    int firstIndex = endIndex - numVertices + 2;  // firstIndex >= 1
                    int lastIndex = endIndex + numVertices;  // lastIndex <= closedWalk.size()
                    closedWalk.erase(closedWalk.begin() + firstIndex, closedWalk.begin() + lastIndex);
                    numClosedWalk = static_cast<int>(closedWalk.size());
                    i = firstIndex - 1;
                }
                else
                {
                    // The filament extends outside the closed walk, so
                    // closedWalk[0] must be part of the filament and is not
                    // a branch point; thus, the entire closed walk is consumed.
                    return true;
                }
            }
        }
        else
        {
            duplicates.insert(std::make_pair(closedWalk[i], i));
        }
    }

    ExtractCycle(closedWalk, manager, tree);
    return true;
}

template <typename Real>
void MinimalCycleBasis<Real>::ExtractCycle(std::vector<Vertex*>& closedWalk,
    HeapManager& manager, PrimitiveTree& tree)
{
    // The processed closed walk is now a simple cycle.
    std::vector<Vertex*> ptrCycle;
    ptrCycle.reserve(closedWalk.size());
    for (auto vertex : closedWalk)
    {
        if (vertex)
        {
            ptrCycle.push_back(vertex);
        }
    }

    int const numVertices = static_cast<int>(ptrCycle.size());
    tree.cycle.resize(numVertices);
    for (int i = 0; i < numVertices; ++i)
    {
        tree.cycle[i] = ptrCycle[i]->name;
    }

    // The clockwise-most edge is always removable.
    Vertex* v0 = ptrCycle[0];
    Vertex* v1 = ptrCycle[1];
    v0->adjacent.erase(v1);
    v1->adjacent.erase(v0);

    // Remove edges while traversing counterclockwise.
    while (v1->adjacent.size() == 1)
    {
        Vertex* adj = *v1->adjacent.begin();
        v1->adjacent.erase(adj);
        adj->adjacent.erase(v1);
        manager.Remove(v1);
        v1 = adj;
    }

    // Remove edges while traversing clockwise.
    while (v0->adjacent.size() == 1)
    {
        v1 = *v0->adjacent.begin();
        v0->adjacent.erase(v1);
        v1->adjacent.erase(v0);
        manager.Remove(v0);
        v0 = v1;
    }

    // Remove the starting vertex if it has become isolated.
    if (v0->adjacent.size() == 0)
    {
        manager.Remove(v0);
    }
}

template <typename Real>
typename MinimalCycleBasis<Real>::Vertex*
MinimalCycleBasis<Real>::GetClockwiseMost(Vertex* vPrev, Vertex* vCurr) const
{
    Vertex* vNext = nullptr;
    bool vCurrConvex = false;
    std::array<Real, 2> dCurr, dNext;
    if (vPrev)
    {
        dCurr[0] = (*vCurr->position)[0] - (*vPrev->position)[0];
        dCurr[1] = (*vCurr->position)[1] - (*vPrev->position)[1];
    }
    else
    {
        dCurr[0] = static_cast<Real>(0);
        dCurr[1] = static_cast<Real>(-1);
    }

    for (auto vAdj : vCurr->adjacent)
    {
        // vAdj is a vertex adjacent to vCurr.  No backtracking is allowed.
        if (vAdj == vPrev)
        {
            continue;
        }

        // Compute the potential direction to move in.
        std::array<Real, 2> dAdj;
        dAdj[0] = (*vAdj->position)[0] - (*vCurr->position)[0];
        dAdj[1] = (*vAdj->position)[1] - (*vCurr->position)[1];

        // Select the first candidate.
        if (!vNext)
        {
            vNext = vAdj;
            dNext = dAdj;
            vCurrConvex = (dNext[0] * dCurr[1] <= dNext[1] * dCurr[0]);
            continue;
        }

        // Update if the next candidate is clockwise of the current
        // clockwise-most vertex.
        if (vCurrConvex)
        {
            if (dCurr[0] * dAdj[1] < dCurr[1] * dAdj[0]
                || dNext[0] * dAdj[1] < dNext[1] * dAdj[0])
            {
                vNext = vAdj;
                dNext = dAdj;
                vCurrConvex = (dNext[0] * dCurr[1] <= dNext[1] * dCurr[0]);
            }
        }
        else
        {
            if (dCurr[0] * dAdj[1] < dCurr[1] * dAdj[0]
                && dNext[0] * dAdj[1] < dNext[1] * dAdj[0])
            {
                vNext = vAdj;
                dNext = dAdj;
                vCurrConvex = (dNext[0] * dCurr[1] < dNext[1] * dCurr[0]);
            }
        }
    }

    return vNext;
}

template <typename Real>
typename MinimalCycleBasis<Real>::Vertex*
MinimalCycleBasis<Real>::GetCounterclockwiseMost(Vertex* vPrev, Vertex* vCurr) const
{
    Vertex* vNext = nullptr;
    bool vCurrConvex = false;
    std::array<Real, 2> dCurr, dNext;
    if (vPrev)
    {
        dCurr[0] = (*vCurr->position)[0] - (*vPrev->position)[0];
        dCurr[1] = (*vCurr->position)[1] - (*vPrev->position)[1];
    }
    else
    {
        dCurr[0] = static_cast<Real>(0);
        dCurr[1] = static_cast<Real>(-1);
    }

    for (auto vAdj : vCurr->adjacent)
    {
        // vAdj is a vertex adjacent to vCurr.  No backtracking is allowed.
        if (vAdj == vPrev)
        {
            continue;
        }

        // Compute the potential direction to move in.
        std::array<Real, 2> dAdj;
        dAdj[0] = (*vAdj->position)[0] - (*vCurr->position)[0];
        dAdj[1] = (*vAdj->position)[1] - (*vCurr->position)[1];

        // Select the first candidate.
        if (!vNext)
        {
            vNext = vAdj;
            dNext = dAdj;
            vCurrConvex = (dNext[0] * dCurr[1] <= dNext[1] * dCurr[0]);
            continue;
        }

        // Select the next candidate if it is counterclockwise of the current
        // counterclockwise-most vertex.
        if (vCurrConvex)
        {
            if (dCurr[0] * dAdj[1] > dCurr[1] * dAdj[0]
                && dNext[0] * dAdj[1] > dNext[1] * dAdj[0])
            {
                vNext = vAdj;
                dNext = dAdj;
                vCurrConvex = (dNext[0] * dCurr[1] <= dNext[1] * dCurr[0]);
            }
        }
        else
        {
            if (dCurr[0] * dAdj[1] > dCurr[1] * dAdj[0]
                || dNext[0] * dAdj[1] > dNext[1] * dAdj[0])
            {
                vNext = vAdj;
                dNext = dAdj;
                vCurrConvex = (dNext[0] * dCurr[1] <= dNext[1] * dCurr[0]);
            }
        }
    }

    return vNext;
}

template <typename Real>
MinimalCycleBasis<Real>::Vertex::Vertex(int inName, std::array<Real, 2> const* inPosition)
    :
    name(inName),
    position(inPosition),
    visited(0)
{
}

template <typename Real>
bool MinimalCycleBasis<Real>::HeapElement::operator== (HeapElement const& element) const
{
    return numAdjacencies == element.numAdjacencies && *position == *element.position;
}

template <typename Real>
bool MinimalCycleBasis<Real>::HeapElement::operator< (HeapElement const& element) const
{
    if (numAdjacencies > 1)
    {
        if (element.numAdjacencies > 1)
        {
            return *position < *element.position;
        }
        else  // element.numAdjacencies == 1
        {
            return false;
        }
    }
    else  // numAdjacencies == 1
    {
        if (element.numAdjacencies > 1)
        {
            return true;
        }
        else  // element.numAdjacencies == 1
        {
            return false;
        }
    }
}

template <typename Real>
bool MinimalCycleBasis<Real>::HeapElement::operator<= (HeapElement const& element) const
{
    return operator<(element) || operator==(element);
}

template <typename Real>
MinimalCycleBasis<Real>::HeapManager::HeapManager(int numElements)
    :
    heap(numElements)
{
}

template <typename Real>
void MinimalCycleBasis<Real>::HeapManager::Insert(
    typename MinimalCycleBasis<Real>::Vertex* vertex)
{
    HeapElement element;
    element.numAdjacencies = static_cast<int>(vertex->adjacent.size());
    element.position = vertex->position;
    records.insert(std::make_pair(vertex, heap.Insert(vertex, element)));
}

template <typename Real>
bool MinimalCycleBasis<Real>::HeapManager::Update(
    typename MinimalCycleBasis<Real>::Vertex* vertex)
{
    auto iter = records.find(vertex);
    if (iter != records.end())
    {
        HeapElement element;
        element.numAdjacencies = static_cast<int>(vertex->adjacent.size());
        if (element.numAdjacencies != iter->second->value.numAdjacencies)
        {
            element.position = iter->second->value.position;
            heap.Update(iter->second, element);
        }
        return true;
    }
    else
    {
        return false;
    }
}

template <typename Real>
bool MinimalCycleBasis<Real>::HeapManager::Remove(
    typename MinimalCycleBasis<Real>::Vertex* vertex)
{
    auto iter = records.find(vertex);
    if (iter != records.end())
    {
        HeapElement element;
        element.numAdjacencies = -1;
        element.position = iter->second->value.position;
        heap.Update(iter->second, element);
        heap.Remove(vertex, element);
        records.erase(vertex);
        return true;
    }
    else
    {
        return false;
    }
}

}
