#ifndef MRR_GRAPH_NODE_HXX_
#define MRR_GRAPH_NODE_HXX_

#include <string>
#include <utility>
#include <vector>

template <typename weight_type = double>
struct dijkstra_graph_node
{
  using index_type = std::size_t;
  using neighbour_type = std::vector<std::pair<weight_type, index_type> >;

  dijkstra_graph_node(
    index_type const& i, std::string const& name_,
    neighbour_type const& neighbours_
  )
    : index(i), name(name_), neighbours(neighbours_)
  {
  }

  index_type index;
  std::string name;
  weight_type distance;
  bool visited;
  std::vector<std::pair<weight_type, index_type> > neighbours;
};


template <typename weight_type = double>
struct dijkstra_weight_compare_less
{
  bool operator()(
    dijkstra_graph_node<weight_type>* a,
    dijkstra_graph_node<weight_type>* b
  )
  {
    return a->distance > b->distance;
  }
};


template <typename WeightType = double>
struct dijkstra_graph
{
  using weight_type = WeightType;
  using node_type = dijkstra_graph_node<weight_type>;
  using index_type = typename node_type::index_type;

  std::vector<node_type> nodes;
};


#endif // #ifndef MRR_GRAPH_NODE_HXX_
