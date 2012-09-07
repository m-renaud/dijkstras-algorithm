#ifndef MRR_DIJKSTRA_HXX_
#define MRR_DIJKSTRA_HXX_

#include <algorithm>
#include <limits>
#include <string>
#include <utility>
#include <vector>


//m=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

namespace mrr {

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-


//m=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
template <typename weight_type>
struct dijkstra_vertex
{
  using index_type = std::size_t;
  using neighbour_type = std::pair<weight_type, index_type>;
  using neighbour_list_type = std::vector<neighbour_type>;

  dijkstra_vertex(
    index_type const& i, std::string const& name_,
    neighbour_list_type const& neighbours_
  )
    : index(i), name(name_), neighbours(neighbours_)
  {
  }

  index_type index;
  std::string name;
  weight_type distance;
  bool visited;
  neighbour_list_type neighbours;
};



//m=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
template <typename weight_type, typename Comparator>
struct dijkstra_weight_compare
{
  bool operator()(
    dijkstra_vertex<weight_type>* a,
    dijkstra_vertex<weight_type>* b
  )
  {
    return Comparator()(b->distance, a->distance);
  }
};



//m=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
template <typename WeightType>
class dijkstra
{
  using weight_type = WeightType;
  using vertex_type = dijkstra_vertex<weight_type>;
  using index_type = typename vertex_type::index_type;
  using vertex_list_type = std::vector<vertex_type>;
  using size_type = typename vertex_list_type::size_type;

  using neighbour_type = typename vertex_type::neighbour_type;

public:

  using neighbour_list_type = typename vertex_type::neighbour_list_type;

  dijkstra()
    : graph_size(0)
  {
  }

  void add_vertex(
    index_type const& i, std::string const& name_,
    neighbour_list_type const& neighbours_
  )
  {
    vertices.push_back(
      vertex_type(
        i, name_, neighbours_
      )
    );

    ++graph_size;
  }

  weight_type find_shortest_path(index_type start, index_type destination)
  {
    vertex_distance_init_value = std::numeric_limits<weight_type>::max();
    reset_graph();
    return find_path<std::less<weight_type> >(start, destination);
  }

  weight_type find_longest_path(index_type start, index_type destination)
  {
    vertex_distance_init_value = std::numeric_limits<weight_type>::min();
    reset_graph();
    return find_path<std::greater<weight_type> >(start, destination);
  }

private:
  void reset_graph();

  template <typename Comparator>
  weight_type find_path(index_type start, index_type destination);

  // Graph representation
  std::vector<vertex_type> vertices;
  size_type graph_size;


  // Path finding variables
  weight_type vertex_distance_init_value;

}; // class dijkstra<WeightType>




//m=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
template <typename WeightType>
void dijkstra<WeightType>::reset_graph()
{
  for(std::size_t i = 0; i < graph_size; ++i)
  {
    vertices[i].distance = vertex_distance_init_value;
    vertices[i].visited = false;
  }
}


//m=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
template <typename WeightType>
template <typename Comparator>
WeightType dijkstra<WeightType>::find_path(
  index_type start, index_type destination
)
{
  vertex_type* current;
  std::vector<vertex_type*> unvisited_nodes(graph_size);
  Comparator comp;

  // For every node in graph, set distance to inifinity
  for(std::size_t i = 0; i < graph_size; ++i)
    unvisited_nodes[i] = &vertices[i];

  vertices[start].distance = 0;

  std::make_heap(
    unvisited_nodes.begin(),
    unvisited_nodes.end(),
    dijkstra_weight_compare<weight_type,Comparator>()
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
      if(!vertices[neighbour.second].visited)
      {
        weight_type alternate_route = current->distance + neighbour.first;

        if(comp(alternate_route, vertices[neighbour.second].distance))
          vertices[neighbour.second].distance = alternate_route;
      }
    }

    std::make_heap(
      unvisited_nodes.begin(),
      unvisited_nodes.end(),
      dijkstra_weight_compare<weight_type, Comparator>()
    );

  } // while(nodes to visit)

  return vertices[destination].distance;
}


//m=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

} // namespace mrr

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

#endif // #ifndef MRR_DIJKSTRA_HXX_
