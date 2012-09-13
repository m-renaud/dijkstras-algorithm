#ifndef MRR_DIJKSTRA_HXX_
#define MRR_DIJKSTRA_HXX_

#include <algorithm>
#include <functional>
#include <limits>
#include <unordered_map>
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
  using neighbour_type = std::pair<weight_type, label_type>;
  using neighbour_list_type = std::vector<neighbour_type>;

  dijkstra_vertex(
    label_type const& label_,
    neighbour_list_type const& neighbours_
  )
    : label(label_), neighbours(neighbours_)
  {
  }

  label_type label;
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
    label_type const& label,
    neighbour_list_type const& neighbours
  )
  {
    vertices.push_back(vertex_type(label, neighbours));
    label_map.insert(std::make_pair(label,graph_size));
    ++graph_size;
  }

  template <
    typename Pred = std::less<weight_type>,
    typename Op = std::plus<weight_type>
  >
  weight_type find_path(
    label_type start, label_type destination,
    weight_type init_val = std::numeric_limits<weight_type>::max(),
    weight_type start_init_val = 0
  )
  {
    vertex_distance_init_value = init_val;
    reset_graph();
    return find_path_impl<Pred,Op>(start, destination, start_init_val);
  }

  weight_type find_shortest_path(
    label_type start, label_type destination,
    weight_type init_val = std::numeric_limits<weight_type>::max()
  )
  {
    vertex_distance_init_value = init_val;
    reset_graph();
    return find_path_impl<std::less<weight_type> >(start, destination);
  }

  weight_type find_longest_path(
    label_type start, label_type destination,
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
  weight_type find_path_impl(
    label_type start, label_type destination,
    weight_type start_init_val = 0
  );

  // Graph representation
  std::vector<vertex_type> vertices;
  size_type graph_size;
  std::unordered_map<label_type, std::size_t> label_map;

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
  label_type start, label_type destination,
  weight_type start_init_val
)
{
  vertex_type* current;
  std::vector<vertex_type*> unvisited_nodes(graph_size);
  Pred comp;
  Op op;

  for(std::size_t i = 0; i < graph_size; ++i)
    unvisited_nodes[i] = &vertices[i];

  vertices[label_map[start]].distance = start_init_val;

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
       || current->label == destination)
      break;

    // Neighbour :: std::pair<weight_type, vertex_type>
    for(neighbour_type& neighbour : current->neighbours)
    {
      std::size_t index = label_map[neighbour.second];
      if(!vertices[index].visited)
      {
        weight_type alternate_route = op(current->distance,neighbour.first);

        if(comp(alternate_route, vertices[index].distance))
          vertices[index].distance = alternate_route;
      }
    }

    std::make_heap(
      unvisited_nodes.begin(),
      unvisited_nodes.end(),
      dijkstra_weight_compare<weight_type,label_type,Pred>()
    );

  } // while(nodes to visit)

  return vertices[label_map[destination]].distance;
}


//m=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

} // namespace mrr

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

#endif // #ifndef MRR_DIJKSTRA_HXX_
