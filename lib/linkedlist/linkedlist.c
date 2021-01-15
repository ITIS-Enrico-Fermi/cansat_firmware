#ifndef LINKED_LIST_C
#define LINKED_LIST_C

#include <stdlib.h>
#include "linkedList.h"

/**
 * @brief createNode implementation
 */
Node* createNode(void* content, Node* previous, Node* next)
{
    Node* result = 
        (Node*)malloc(sizeof(Node));

    result->content = content;
    
    // Linking the nodes
    llLinkThree(previous, result, next);

    return result;
}

/**
 * @brief Linked list add implementation
 */
Node* llAdd(Node* start, void* content)
{
    // Getting the last node of the
    // list
    Node* end = llGetLast(start);

    if (end == NULL) return NULL;

    // Creating the new node
    Node* new = 
        createNode(content, end, NULL);

    // Adding the new node to the
    // list
    end->next = new;
    
    return new;
}

/**
 * @brief Inserts a node at the specified index
 * of the linked list.
 * 
 * @param start The beginning node of the list.
 * @param element The element to add.
 * @param index Where to insert the node.
 */
void llInsertAt(Node* start, Node* element, int index)
{
    if (element == NULL) return;

    // Getting the node at the
    // requested index
    Node* target = llElementAt(start, index);

    if (target == NULL) return;

    // Linking all together
    llLinkThree(target, element, target->next);    
}

/**
 * @brief Returns a pointer to a node at
 * the requested index of the linked list.
 */
Node* llElementAt(Node* start, int index)
{
    if (start == NULL || index < 0)
        return NULL;

    register int i;
    register Node* current;
    for (
        current = start, i = 0
        ; 
        current != NULL && i != index
        ; 
        current = current->next, i++)
        ;

    return current;
}

/**
 * @brief Returns a pointer to the last 
 * node of a linked list.
 */
inline Node* llGetLast(Node* start)
{
    if (start == NULL) return NULL;

    // Current is used to cycle though
    // the elements
    register Node* current;
    for (
        current = start; 
        current->next != NULL; 
        current = current->next)
        ;
    
    return current;
}

/**
 * @brief Returns a pointer to the first 
 * node of a linked list.
 */
Node* llGetFirst(Node* last)
{
    if (last == NULL) return NULL;

    // Current is used to cycle though
    // the elements
    register Node* current;
    for (
        current = last; 
        current->previous != NULL; 
        current = current->previous)
        ;
    
    return current;
}

/**
 * @brief Returns the length of a linked list
 * 
 * @param start The starting node
 */
int llLength(Node* start)
{
    if (start == NULL)
        return 0;

    int result;
    register Node* current;
    for (
        current = start, result = 1;
        current->next != NULL;
        current = current->next, result++)
        ;

    return result;
}


/**
 * @brief Removes a node at the specified index.
 */
void llRemoveAt(Node** start, int index, bool freeContent)
{
    if (*start == NULL) return;

    // Getting the node
    Node* node = 
        llElementAt(*start, index);

    if (node == NULL) return;

    Node* prev = node->previous;
    Node* next = node->next;
    
    // Linking the previous to the 
    // next and the next to the previous
    llLinkTwo(prev, next);
    
    if (freeContent)
        free(node->content);
    
    // Freeing the memory used by the node
    free(node);
    
    if (index == 0) *start = next;
}


/**
 * @brief Removes a target node from the a 
 * linked list.
 */
void llRemove(Node** start, Node* target, bool freeContent)
{
    if (*start == NULL || target == NULL) return;
    
    register Node* current;
    for (
        current = *start;
        current != NULL;
        current = current->next)
    {
        if (current == target)
        {
            // If the first node was removed,
            // updating the list's head
            if (*start == target)
                *start = (*start)->next;
            
            // Otherwise, linking the two nodes
            else 
                llLinkTwo(
                    target->previous,
                    target->next);
            
            if (freeContent)
                free(target->content);
            
            free(target);
            return;
        }
    }
}


/**
 * @brief Removes the last node from the linked list.
 */
void llRemoveLast(Node** start, bool freeContent)
{
    if (*start == NULL) return;
  
    Node* last = llGetLast(*start);

    if (last == NULL) return;

    // If the list only has one node,
    // setting it to null
    if (*start == last)
        *start = NULL;
    
    llLinkTwo(last->previous, NULL);
    
    if (freeContent)
        free(last->content);
    
    free(last);
}

/**
 * @brief Links two nodes together
 */
inline void llLinkTwo(Node* previous, Node* next)
{
    if (previous != NULL)
        previous->next = next;

    if (next != NULL)
        next->previous = previous;
}

/**
 * @brief Links three nodes together.
 */
inline void llLinkThree(Node* previous, register Node* middle, Node* next)
{
    if (middle == NULL) return;

    middle->previous = previous;
    middle->next     = next;

    if (next != NULL)
        next->previous = middle;

    if (previous != NULL)
        previous->next = middle;
}

/**
 * @brief Clears a linked list by deleting all of
 * its nodes.
 */
void llClear(Node** start, bool freeContent)
{
    if (*start == NULL) return;
  
    register Node* current;
    register Node* next;
    
    for (
        current = *start, next = current->next
        ;
        current != NULL
        ;
        current = next, 
        next = (current == NULL ? current->next : NULL))
    {
        if (freeContent)
            free(current->content);
        
        free(current);
    }
    
    *start = NULL;
}

#endif // !LINKED_LIST_C