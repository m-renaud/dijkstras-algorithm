#include <iostream>
#include <string>
#include <functional>

#include "dijkstra.hxx"

int main()
{
  // We want a directed graph with edge weight represented as doubles
  // The type that we use to identify the vertices will be a std::string
  mrr::dijkstra<double,std::string> d;

  // Add each vertex, specifying it's name and it's neighbours
  // as vector<pair<edge_weight,name>>
  d.add_vertex(
    "A",
    {
      {0.1,"B"},
      {0.3,"C"}
    }
  );

  d.add_vertex(
    "B",
    {
      {0.1,"A"},
      {0.1,"C"},
      {0.4,"D"}
    }
  );

  d.add_vertex(
    "C",
    {
      {0.3,"A"},
      {0.1,"B"},
      {0.1,"D"}
    }
  );

  d.add_vertex(
    "D",
    {
      {0.4,"B"},
      {0.1,"C"}
    }
  );

  std::cout << d.find_shortest_path("A","D") << std::endl;
  std::cout << d.find_shortest_path("B","D") << std::endl;

  // Now treat them like probabilities and find the best probability path
  // - We want the highest probability so use std::greater for comparison
  // - We want to multiply the edge weight to find the new ``distance"
  // - Our initial distance will be 1

  using std::greater;
  using std::multiplies;
  std::cout << d.find_path<greater,multiplies>("A", "D", 1) << std::endl;

  return 0;
}
