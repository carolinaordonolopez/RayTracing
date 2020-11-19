/* list.h
 *
 * Realidad Virtual y Aumentada.
 *
 * Practice 2.
 * Ray tracing.
 *
 * Jose Pascual Molina Masso.
 * Escuela Superior de Ingenieria Informatica de Albacete.
 */

#ifndef _LIST_H_
#define _LIST_H_


/**************/
/* List class */
/**************/

template <class element>
class List {

public:

  /* Constructor and destructor */
  List();
  ~List();

  /* Functions */
  void Add(element e);  // Insert at the front of the list
  element First();      // Returns the first element
  element Next();       // Returns the next element
  int Length();         // Length of list  

private:

  class Node {
  public:
    element e;
	Node* next;
  };

  Node *first, *next;
  int count;
};

/* Constructor */
template <class element>
inline List<element>::List() {
  first = NULL;
  next = NULL;
  count = 0;
}

/* Destructor */
template <class element>
inline List<element>::~List() {
  while (first != NULL) {
	next = first->next;
	delete first;
	first = next;
  }
}

/* Insert at the front of the list */
template <class element>
inline void List<element>::Add(element e) {
  Node* nptr = new Node;

  nptr->e = e;
  nptr->next = first;
  first = nptr;
  count++;
}

/* Returns the first object */
template <class element>
inline element List<element>::First() {
  if (first != NULL)
    next = first->next;
  return first->e;
}

/* Returns the next object */
template <class element>
inline element List<element>::Next() {
  if (next != NULL) {
    Node* nptr = next;
    next = next->next;
	return nptr->e;
  }
  return NULL;
}

/* Length of list */
template <class element>
inline int List<element>::Length() {
  return count;
}


#endif  // !defined _LIST_H_
