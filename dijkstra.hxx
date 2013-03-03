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
  bool seen;
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
    if (a->seen == false && b->seen == false)
      return false;
    else if (a->seen == false && b->seen == true)
      return true;
    else if (a->seen == true && b->seen == false)
      return false;
    else
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
    template <typename...> class Pred = std::less,
    template <typename...> class  Op = std::plus
  >
  weight_type find_path(
    label_type start, label_type destination,
    weight_type start_init_val = 0
  )
  {
    return find_path_impl<Pred<weight_type>, Op<weight_type> >(
      start, destination, start_init_val
    );
  }

  weight_type find_shortest_path(label_type start, label_type destination)
  {
    return find_path<>(start, destination);
  }

  weight_type find_longest_path(label_type start, label_type destination)
  {
    return find_path<std::greater>(start, destination);
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

}; // class dijkstra<WeightType>




//m=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
template <typename WeightType, typename LabelType>
void dijkstra<WeightType,LabelType>::reset_graph()
{
  for (std::size_t i = 0; i < graph_size; ++i)
  {
    vertices[i].visited = false;
    vertices[i].seen = false;
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

  reset_graph();

  for (std::size_t i = 0; i < graph_size; ++i)
    unvisited_nodes[i] = &vertices[i];

  vertices[label_map[start]].distance = start_init_val;
  vertices[label_map[start]].visited = true;
  vertices[label_map[start]].seen = true;

  std::make_heap(
    unvisited_nodes.begin(),
    unvisited_nodes.end(),
    dijkstra_weight_compare<weight_type,label_type,Pred>()
  );

  while (!unvisited_nodes.empty())
  {
    current = unvisited_nodes.front();
    std::pop_heap(unvisited_nodes.begin(), unvisited_nodes.end());
    unvisited_nodes.pop_back();
    current->visited = true;

    if (! current->seen || current->label == destination)
      break;

    // Neighbour :: std::pair<weight_type, vertex_type>
    for (neighbour_type& neighbour : current->neighbours)
    {
      std::size_t index = label_map[neighbour.second];
      vertex_type& vertex = vertices[index];

      if (! vertex.visited)
      {
        weight_type alternate_route = op(current->distance, neighbour.first);

        if (! vertex.seen || comp(alternate_route, vertex.distance))
          vertex.distance = alternate_route;

        vertex.seen = true;
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
