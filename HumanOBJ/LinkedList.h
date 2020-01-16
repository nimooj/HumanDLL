#pragma once
#include <stdlib.h>

template <typename T>
class Node {
public:
	Node() {
		next = nullptr;
	}
	Node(T n) {
		next = nullptr;
		value = n;
	}
	~Node() {}

	T value;
	Node<T> *next;
};

template <typename T> 
class LinkedList {
public:
	int length;
	Node<T> *head;
	Node<T> *curr;

	LinkedList() {
		head = new Node<T>();
		curr = head;
		length = 0;
	}
	~LinkedList() {
	}

	void append(T n) {
		Node<T> *node = new Node<T>(n);

		if (length == 0) { // Insert First element
			head = node;
			curr = head;
		}
		else {
			curr->next = node;
			curr = node;
		}
		length+=1;
	}

	void append(Node<T> n) {
		Node<T> *node = n;
		if (length == 0) {
			head = node;
			curr = head;
		}
		else {
			curr->next = node;
			curr = node;
		}
		length += 1;
	}

	void insert(int location, Node<T> n) {
		curr = head;
		for (int i = 0; i < location; i++) {
			curr = curr->next;
		}
		n->next = curr->next;
		curr->next = n;
	}

	void remove() {
		if (length == 0) {
			return;
		}

		Node<T> t = head;
		while (t->next) {
			curr = t;
			t = t->next;
		}

		curr->next = nullptr;
		delete t;
		length--;
	}
};