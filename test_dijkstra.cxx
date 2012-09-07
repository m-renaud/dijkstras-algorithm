#include <iostream>

#include "dijkstra.hxx"

int main()
{
  using neighbour_type = std::vector<std::pair<int, std::size_t> >;
  dijkstra_graph<int> g;

  g.nodes.push_back(
    dijkstra_graph_node<int>(
      0,"",
      neighbour_type{
        std::make_pair(1,1),
        std::make_pair(3,2)
      }
    )
  );

  g.nodes.push_back(
    dijkstra_graph_node<int>(
      1,"",
      neighbour_type{
        std::make_pair(1,0),
        std::make_pair(1,2),
        std::make_pair(4,3)
      }
    )
  );

  g.nodes.push_back(
    dijkstra_graph_node<int>(
      2,"",
      neighbour_type{
        std::make_pair(3,0),
        std::make_pair(1,1),
        std::make_pair(1,3)
      }
    )
  );

  g.nodes.push_back(
    dijkstra_graph_node<int>(
      3,"",
      neighbour_type{
        std::make_pair(4,1),
        std::make_pair(1,2)
        }
    )
  );

  std::cout << dijkstra(g,0,3) << std::endl;
  std::cout << dijkstra(g,1,3) << std::endl;
}
