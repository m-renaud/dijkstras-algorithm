#include <iostream>

#include "dijkstra.hxx"

int main()
{
  // using neighbour_type = std::vector<std::pair<int, std::size_t> >;
  // dijkstra_graph<int> g;

  mrr::dijkstra<int> d;

  d.add_vertex(
    0,"",
    {
      std::make_pair(1,1),
      std::make_pair(3,2)
    }
  );

  d.add_vertex(
    1,"",
    {
      std::make_pair(1,0),
      std::make_pair(1,2),
      std::make_pair(4,3)
    }
  );

  d.add_vertex(
    2,"",
    {
      std::make_pair(3,0),
      std::make_pair(1,1),
      std::make_pair(1,3)
    }
  );

  d.add_vertex(
    3,"",
    {
      std::make_pair(4,1),
      std::make_pair(1,2)
    }
  );

  std::cout << d.find_shortest_path(0,3) << std::endl;
  std::cout << d.find_shortest_path(1,3) << std::endl;
}
