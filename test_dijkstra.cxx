#include <iostream>
#include <string>
#include <functional>

#include "dijkstra.hxx"

int main()
{
  // We want a directed graph with edge weight represented as doubles
  // The type that we use to identify the vertices will be a std::string
  mrr::dijkstra<double,std::string> d;
  mrr::dijkstra<double,std::string>::neighbour_list_type n;

  // Add each vertex, specifying it's name and it's neighbours
  // as vector<pair<edge_weight,name>>
  n.push_back(std::make_pair(0.1,"B"));
  n.push_back(std::make_pair(0.3,"C"));
  d.add_vertex(
    "A",
    n
  );

  n.clear();
  n.push_back(std::make_pair(0.1,"A"));
  n.push_back(std::make_pair(0.1,"C"));
  n.push_back(std::make_pair(0.4,"D"));

  d.add_vertex(
    "B",
    n
  );

  n.clear();
  n.push_back(std::make_pair(0.3,"A"));
  n.push_back(std::make_pair(0.1,"B"));
  n.push_back(std::make_pair(0.1,"D"));

  d.add_vertex(
    "C",
    n
  );

  n.clear();
  n.push_back(std::make_pair(0.4,"B"));
  n.push_back(std::make_pair(0.1,"C"));

  d.add_vertex(
    "D",
    n
  );

  std::cout << d.find_shortest_path("A","D") << std::endl;
  std::cout << d.find_shortest_path("B","D") << std::endl;

  // Now treat them like probabilities and find the best probability path
  // - We want the highest probability so use std::greater for comparison
  // - We want to multiply the edge weight to find the new ``distance"
  // - Our infinity value should be -inf
  // - Our initial distance will be 1
  std::cout << d.find_path<std::greater<double>,std::multiplies<double> >(
    "A","D",-std::numeric_limits<double>::max(), 1
  );

  endl(std::cout);

  return 0;
}
