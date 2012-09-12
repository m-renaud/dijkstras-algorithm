#include <iostream>
#include <string>

#include "dijkstra.hxx"

int main()
{
  mrr::dijkstra<int,std::string> d;

  d.add_vertex(
    "A",
    {
      {1,"B"},
      {3,"C"}
    }
  );

  d.add_vertex(
    "B",
    {
      {1,"A"},
      {1,"C"},
      {4,"D"}
    }
  );

  d.add_vertex(
    "C",
    {
      {3,"A"},
      {1,"B"},
      {1,"D"}
    }
  );

  d.add_vertex(
    "D",
    {
      {4,"B"},
      {1,"C"}
    }
  );

  std::cout << d.find_shortest_path("A","D") << std::endl;
  std::cout << d.find_shortest_path("B","D") << std::endl;
}
