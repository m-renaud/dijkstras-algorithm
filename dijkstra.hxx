#ifndef MRR_DIJKSTRA_HXX_
#define MRR_DIJKSTRA_HXX_

#include <algorithm>
#include <limits>
#include <vector>

#include "graph_node.hxx"

template <
  typename dijkstra_graph_type,
  typename Comparator = dijkstra_weight_compare_less<
    typename dijkstra_graph_type::weight_type
  >
>
typename dijkstra_graph_type::weight_type dijkstra(
  dijkstra_graph_type& graph,
  typename dijkstra_graph_type::index_type const& source,
  typename dijkstra_graph_type::index_type const& destination
)
{
  using index_type = typename dijkstra_graph_type::index_type;
  using node_type = typename dijkstra_graph_type::node_type;
  using weight_type = typename dijkstra_graph_type::weight_type;

  using neighbour_type = std::pair<weight_type,index_type>;

  auto graph_size = graph.nodes.size();
  node_type* current;
  std::vector<node_type*> unvisited_nodes(graph_size);


  // For every node in graph, set distance to inifinity
  for(std::size_t i = 0; i < graph_size; ++i)
  {
    graph.nodes[i].distance = std::numeric_limits<weight_type>::max();
    graph.nodes[i].visited = false;
    unvisited_nodes[i] = &graph.nodes[i];
  }
  graph.nodes[source].distance = 0;

  std::make_heap(
    unvisited_nodes.begin(),
    unvisited_nodes.end(),
    dijkstra_weight_compare_less<weight_type>()
  );



  while(!unvisited_nodes.empty())
  {
    current = unvisited_nodes.front();
    std::pop_heap(unvisited_nodes.begin(), unvisited_nodes.end());
    unvisited_nodes.pop_back();
    current->visited = true;

    if(current->distance == std::numeric_limits<weight_type>::max()
       || current->index == destination)
      break;

    // Neighbour :: std::pair<weight_type, node_type>
    for(neighbour_type& neighbour : current->neighbours)
    {
      if(!graph.nodes[neighbour.second].visited)
      {
        weight_type alternate_route = current->distance + neighbour.first;

        if(alternate_route < graph.nodes[neighbour.second].distance)
          graph.nodes[neighbour.second].distance = alternate_route;
      }
    }

    std::make_heap(
      unvisited_nodes.begin(),
      unvisited_nodes.end(),
      dijkstra_weight_compare_less<weight_type>()
    );

  } // while(nodes to visit)

  return graph.nodes[destination].distance;
}


#endif // #ifndef MRR_DIJKSTRA_HXX_
