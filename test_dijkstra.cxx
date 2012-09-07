#include <iostream>

#include "dijkstra.hxx"

int main()
{
  mrr::dijkstra<int> d;

  d.add_vertex(
    0,"A",
    {
      {1,1},
      {3,2}
    }
  );

  d.add_vertex(
    1,"B",
    {
      {1,0},
      {1,2},
      {4,3}
    }
  );

  d.add_vertex(
    2,"C",
    {
      {3,0},
      {1,1},
      {1,3}
    }
  );

  d.add_vertex(
    3,"D",
    {
      {4,1},
      {1,2}
    }
  );

  std::cout << d.find_shortest_path(0,3) << std::endl;
  std::cout << d.find_shortest_path(1,3) << std::endl;
}
