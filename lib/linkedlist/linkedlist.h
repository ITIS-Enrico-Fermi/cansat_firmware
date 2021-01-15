#ifndef LINKED_LIST_H
#define LINKED_LIST_H


#include "stdbool.h"


/**
 * @brief Represents a node inside a 
 * linked list.
 */
typedef struct Node
{
    /**
     * @brief A pointer to the content of
     * this Node. Must be casted before 
     * accessing.
     */
    void* content;

    /**
     * @brief The previous node.
     */
    struct Node* previous;

    /**
     * @brief The next node.
     */
    struct Node* next;
} Node;

/**
 * @brief Links two nodes together
 */
void llLinkTwo(Node* previous, Node* next);

/**
 * @brief Links three nodes together
 */
void llLinkThree(Node* previous, register Node* middle, Node* next);


/**
 * @brief Allocates a new Node and returns a pointer
 * to it.
 * 
 * @param content A pointer to this node's content.
 * @param previous A pointer to the previous node.
 * @param next A pointer to the next node.
 */
Node* createNode(void* content, Node* previous, Node* next);

/**
 * @brief Adds a node at the end of a linked 
 * list.
 * 
 * @param start The beginning element of the list.
 * @param content The content of the new Node.
 * @returns The new node.
 */
Node* llAdd(Node* start, void* content);

/**
 * @brief Inserts a node at the specified index
 * of the linked list.
 * 
 * @param start The beginning node of the list.
 * @param element The element to add.
 * @param index Where to insert the node.
 */
void llInsertAt(Node* start, Node* element, int index);

/**
 * @brief Returns a pointer to a node at
 * the requested index of the linked list.
 */
Node* llElementAt(Node* start, int index);

/**
 * @brief Returns a pointer to the last 
 * node of a linked list.
 */
Node* llGetLast(Node* start);

/**
 * @brief Returns a pointer to the first 
 * node of a linked list.
 */
Node* llGetFirst(Node* last);

/**
 * @brief Returns the length of a linked list
 * 
 * @param start The starting node
 */
int llLength(Node* start);

/**
 * @brief Removes a node at the specified index.
 *
 * @param start 
 *      A pointer to a pointer that points to the 
 *      beginning node.
 *      Used because so the function can automatically 
 *      adjust the first node if it is removed.
 *
 * @param index
 *      The index of the element that needs to be removed.
 * 
 * @param freeContent
 *      Tells whether the function should call free on the
 *      Node's content too.
 */
void llRemoveAt(Node** start, int index, bool freeContent);

/**
 * @brief Removes a target node from the a 
 * linked list.
 *
 * @param start 
 *      A pointer to a pointer that points to the 
 *      beginning node.
 *      Used because so the function can automatically 
 *      adjust the first node if it is removed.
 *
 * @param target 
 *      A pointer to the node that needs to be removed.
 *
 * @param freeContent
 *      Tells whether the function should call free on the
 *      Node's content too.
 */
void llRemove(Node** start, Node* target, bool freeContent);   

/**
 * @brief Removes the last node from the linked list.
 *
 * @param start 
 *      A pointer to a pointer that points to the 
 *      beginning node.
 *      Used because so the function can automatically 
 *      set the first node to NULL if it is removed.
 *
 * @param freeContent
 *      Tells whether the function should call free on the
 *      Node's content too.
 */
void llRemoveLast(Node** start, bool freeContent);

/**
 * @brief Clears a linked list by deleting all of
 * its nodes.
 * After this, a new call to createNode(...) will be needed.
 *
 * @param start 
 *      A pointer to a pointer that points to the 
 *      beginning node.
 *      Used because so the function can automatically 
 *      set the beginning node to NULL.
 * 
 * @param freeContent
 *      Tells whether the function should call free on the
 *      Node's content too.
 */
void llClear(Node** start, bool freeContent);


#endif // !LINKED_LIST_H