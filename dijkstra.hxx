#ifndef MRR_DIJKSTRA_HXX_
#define MRR_DIJKSTRA_HXX_

#include <algorithm>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <vector>


//m=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

namespace mrr {

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-


//m=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
template <typename WeightType, typename LabelType>
struct dijkstra_vertex
{
  using weight_type = WeightType;
  using label_type = LabelType;
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
template <typename WeightType, typename LabelType, typename Comparator>
struct dijkstra_weight_compare
{
  bool operator()(
    dijkstra_vertex<WeightType,LabelType>* a,
    dijkstra_vertex<WeightType,LabelType>* b
  )
  {
    return Comparator()(b->distance, a->distance);
  }
};



//m=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
template <typename WeightType, typename LabelType = std::size_t>
class dijkstra
{
  using weight_type = WeightType;
  using label_type = LabelType;
  using vertex_type = dijkstra_vertex<weight_type, label_type>;
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

  template <
    typename Pred = std::less<weight_type>,
    typename Op = std::plus<weight_type>
  >
  weight_type find_path(
    index_type start, index_type destination,
    weight_type init_val = std::numeric_limits<weight_type>::max()
  )
  {
    vertex_distance_init_value = init_val;
    reset_graph();
    return find_path_impl<Pred,Op>(start, destination);
  }

  weight_type find_shortest_path(
    index_type start, index_type destination,
    weight_type init_val = std::numeric_limits<weight_type>::max()
  )
  {
    vertex_distance_init_value = init_val;
    reset_graph();
    return find_path_impl<std::less<weight_type> >(start, destination);
  }

  weight_type find_longest_path(
    index_type start, index_type destination,
    weight_type init_val = std::numeric_limits<weight_type>::min()
  )
  {
    vertex_distance_init_value = init_val;
    reset_graph();
    return find_path_impl<std::greater<weight_type> >(start, destination);
  }

private:
  void reset_graph();

  template <typename Pred, typename Op = std::plus<weight_type> >
  weight_type find_path_impl(index_type start, index_type destination);

  // Graph representation
  std::vector<vertex_type> vertices;
  size_type graph_size;


  // Path finding variables
  weight_type vertex_distance_init_value;

}; // class dijkstra<WeightType>




//m=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
template <typename WeightType, typename LabelType>
void dijkstra<WeightType,LabelType>::reset_graph()
{
  for(std::size_t i = 0; i < graph_size; ++i)
  {
    vertices[i].distance = vertex_distance_init_value;
    vertices[i].visited = false;
  }
}


//m=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
template <typename WeightType, typename LabelType>
template <typename Pred, typename Op>
WeightType dijkstra<WeightType,LabelType>::find_path_impl(
  index_type start, index_type destination
)
{
  vertex_type* current;
  std::vector<vertex_type*> unvisited_nodes(graph_size);
  Pred comp;
  Op op;

  for(std::size_t i = 0; i < graph_size; ++i)
    unvisited_nodes[i] = &vertices[i];

  vertices[start].distance = 0;

  std::make_heap(
    unvisited_nodes.begin(),
    unvisited_nodes.end(),
    dijkstra_weight_compare<weight_type,label_type,Pred>()
  );


  while(!unvisited_nodes.empty())
  {
    current = unvisited_nodes.front();
    std::pop_heap(unvisited_nodes.begin(), unvisited_nodes.end());
    unvisited_nodes.pop_back();
    current->visited = true;

    if(current->distance == vertex_distance_init_value
       || current->index == destination)
      break;

    // Neighbour :: std::pair<weight_type, vertex_type>
    for(neighbour_type& neighbour : current->neighbours)
    {
      if(!vertices[neighbour.second].visited)
      {
        weight_type alternate_route = op(current->distance,neighbour.first);

        if(comp(alternate_route, vertices[neighbour.second].distance))
          vertices[neighbour.second].distance = alternate_route;
      }
    }

    std::make_heap(
      unvisited_nodes.begin(),
      unvisited_nodes.end(),
      dijkstra_weight_compare<weight_type,label_type,Pred>()
    );

  } // while(nodes to visit)

  return vertices[destination].distance;
}


//m=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

} // namespace mrr

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

#endif // #ifndef MRR_DIJKSTRA_HXX_
