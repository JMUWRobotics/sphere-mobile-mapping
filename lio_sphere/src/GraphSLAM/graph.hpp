#pragma once

#include <vector>
#include <string>
#include <fstream>

#include <boost/graph/adjacency_list.hpp>

/* Boost graph declaration */
using boost::edge_weight_t;
typedef boost::adjacency_list <
    boost::listS, boost::vecS, boost::undirectedS,
    boost::no_property,
    boost::property < edge_weight_t, double > >
  graph_t;

/**
 * @brief This class represent a directed network.
 *        Each node corresponds to a laser scan.
 */
class Graph {

public:

  Graph();
  Graph(int nrScans, bool loop);

  int getLink(int i, int fromTo);
  void addLink(int i, int j);

  void setNrScans(int _nrScans);
  int getNrScans();
  int getNrLinks();

  int getStart();
  int getEnd();

  friend std::ostream& operator<<(std::ostream& os, Graph* gr);

private:
  /**
   * The basic network structure
   */
  std::vector <int> from, to;

  /**
   * The number of scans contained in this Graph
   */
  int nrScans;

  /**
   * The very first Scan
   */
  int start;
};
