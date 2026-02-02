#include "graph.hpp"

#include <fstream>
#include <iostream>

/**
 * Constructor to create an empty graph
 */
Graph::Graph()
{
  nrScans = 0;
  start = 0;
}

/**
 * Constructor builds a minimally connected Graph with a given number of scans.
 * The loop can optionally be closed,
 * so that the first will be connected to the last scan.
 *
 * @param nScans The number of Scans
 * @param loop Specifies whether the first and last scan should be linked
 */
Graph::Graph(int nScans, bool loop)
{
  int nrLinks = 0;
  start = 0;
  nrScans = nScans;

  if (loop) {
    nrLinks = nScans;
  } else {
    nrLinks = nScans - 1;
  }

  for(int i = 0 ;i < nrLinks; i++){
    from.push_back(i);
    if (loop) {
      to.push_back(i != nrLinks - 1 ?  i + 1 : 0) ;
    } else {
      to.push_back(i + 1);
    }
  }
}



/**
 * Returns the specified link
 *
 * @param i The i-th link
 * @param fromTo 0 is the outgoing node and 1 the ingoing
 * @return An integer for the node
 */
int Graph::getLink(int i, int fromTo)
{
  if (i >= 0 && i < (int)from.size()) {
    if (fromTo == 0) return from[i];
     else return to[i];
  }
  return 0;
}


/**
 * adds a link to a graph
 *
 * @param i from node
 * @param j to node
 */
void Graph::addLink(int i, int j)
{
  int present = 0;
  for (unsigned int iterator = 0; iterator < from.size(); iterator++) {
    if (from[iterator] == i) present++;
    if (to[iterator] == i) present++;
  }
  if (present == 0) nrScans++;
  present = 0;
  for (unsigned int iterator = 0; iterator < from.size(); iterator++) {
    if (from[iterator] == j) present++;
    if (to[iterator] == j) present++;
  }
  if (present == 0) nrScans++;

  from.push_back(i);
  to.push_back(j);
}


/**
 * Returns the number of links
 * @return number of links
 */
int Graph::getNrLinks()
{
  return from.size();
}

/**
 * Returns the number of scans
 * @return number of scans
 */
int Graph::getNrScans()
{
  return nrScans;
}

/**
 * sets the number of scans
 * @param number of scans
 */
void Graph::setNrScans(int _nrScans)
{
  nrScans = _nrScans;
}

/**
 * Returns the number of the first scan
 * @return number of the first scan
 */
int Graph::getStart()
{
  return start;
}

/**
 * Returns the number of the last scan
 * @return number of the last scan
 */
int Graph::getEnd()
{
  return start+nrScans - 1;
}

/**
 * Prints out the Graph nicely formatted
 * @param os the stream to print to
 * @param gr which Graph to print
 * @return the resulting output stream
 */
std::ostream& operator<<(std::ostream& os, Graph* gr)
{
  for(int i = 0; i < (int)gr->from.size() ; i++){
    os << "( " << gr->getLink(i,0) << " - " << gr->getLink(i,1) << " )" << std::endl;
  }
  return os;
}
