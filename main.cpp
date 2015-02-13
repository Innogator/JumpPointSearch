#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <malloc.h>
#include <cstdio>
#include <math.h>

#define min(a, b) (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) > (b)) ? (a) : (b))

// Circular Doubly Linked List that contains a list of all open nodes
struct open_list {
	struct open_list *left;
	struct open_list *right;
	struct node *list_node;
};

// mergesort the list by a reference to the head
struct open_list *open_list_sort(struct open_list *list);

// monitor all information about a node on the grid
struct node {
	int x;
	int y;
	bool traversable;
	int g;
	int h;
	int f;
	bool opened;
	bool closed;
	struct node *parent;
};

// the grid representing the map
struct grid {
	int width, height;
	struct node **nodes;
};

/* Circular Doubly Linked List that holds X & Y coordinates instead of full nodes */
struct neighbor_xy_list {
	struct neighbor_xy_list *left;
	struct neighbor_xy_list *right;
	int x;
	int y;
};

/* contains all neighbors of a node */
struct neighbor_list {
	struct neighbor_list *left;
	struct neighbor_list *right;
	struct node *neighbor_node;
};

/*
 * Function Definitions
 */

struct neighbor_list *new_neighbor_list();
struct open_list *open_list_new();
struct open_list *open_list_insert_right(struct open_list *list, struct node *data);
struct open_list *open_list_delete(struct open_list *list);
struct neighbor_list *insert_right(struct neighbor_list *list, struct node *data);
struct node createNode(int x, int y, bool traversable);
struct grid createGrid(int width, int height, bool **matrix);
struct node **_buildNodes(int width, int height, bool **matrix);
struct node *getNodeAt(struct grid *gd, int x, int y);
struct neighbor_xy_list *backtrace(struct node *activeNode);
struct neighbor_xy_list *findPath(struct grid *gd, int startX, int startY, int endX, int endY);
struct neighbor_xy_list *smooth_path(struct grid *gd, struct neighbor_xy_list *head);
struct neighbor_list *getNeighbors(struct grid *gd, struct node *nd);
struct neighbor_xy_list *neighbor_xy_new();
struct neighbor_xy_list *neighbor_xy_insert_right(struct neighbor_xy_list *list, int x, int y);
struct neighbor_xy_list *_findNeighbors(struct grid *gd, struct node *activeNode);
void printGrid(struct grid *gd);
void printNodeInfo(struct node *n);
void printSolution(struct grid *gd, struct neighbor_xy_list *path);
void listNeighbors(struct neighbor_list *list);
void listOpenList(struct open_list *list);
int cmp(struct open_list *one, struct open_list *two);
void open_list_clean(struct open_list *head);
void clean_neighbor_list(struct neighbor_list *head);
bool istraversableAt(struct grid *gd, int x, int y);
bool isInside(struct grid *gd, int x, int y);
void settraversableAt(struct grid *gd, int x, int y, bool traversable);
int euclidean(int dx, int dy);
int manhattan(int dx, int dy);
int *_jump(struct grid *gd, int x, int y, int px, int py, struct node *endNode);
void _get_successors(struct grid *gd, struct node *activeNode, struct open_list *current, struct node *endNode);
void neighbor_xy_clean(struct neighbor_xy_list *head);

/*
 * Return euclidean heuristic
 */
int euclidean(int dx, int dy)
{
	double distance = sqrt((double)(dx * dx + dy * dy)) * 10.0;
	int idistance = (int)distance;
	return idistance;
}

/*
 * Return manhattan heuristic
 */
int manhattan(int dx, int dy)
{
	int distance = (dx + dy) * 10;
	return distance;
}

/*
 * Perform a jump from one point to another if possible
 */
int *_jump(struct grid *gd, int x, int y, int px, int py, struct node *endNode)
{
	int dx = x - px;
	int dy = y - py;
	int *jx, *jy;
	if (!istraversableAt(gd, x, y)) {
		return NULL;
	}
	else if (getNodeAt(gd, x, y) == endNode) {
		int *i = (int *) malloc(2 * sizeof(int));
		i[0] = x;
		i[1] = y;
		return i;
	}

	if (dx != 0 && dy != 0) {
		if ((istraversableAt(gd, (x - dx), (y + dy)) && !istraversableAt(gd, (x - dx), y)) ||
		    (istraversableAt(gd, (x + dx), (y - dy)) && !istraversableAt(gd, x, (y - dy)))) {
			int *i = (int *) malloc(2 * sizeof(int));
			i[0] = x;
			i[1] = y;
			return i;
		}
	}
	else {
		if (dx != 0) {
			if ((istraversableAt(gd, (x + dx), (y + 1)) && !istraversableAt(gd, x, (y + 1))) ||
			    (istraversableAt(gd, (x + dx), (y - 1)) && !istraversableAt(gd, x, (y - 1)))) {
				int *i = (int *) malloc(2 * sizeof(int));
				i[0] = x;
				i[1] = y;
				return i;
			}
		}
		else {
			if ((istraversableAt(gd, (x + 1), (y + dy)) && !istraversableAt(gd, (x + 1), y)) ||
			    (istraversableAt(gd, (x - 1), (y + dy)) && !istraversableAt(gd, (x - 1), y))) {
				int *i = (int *) malloc(2 * sizeof(int));
				i[0] = x;
				i[1] = y;
				return i;
			}
		}
	}

	if (dx != 0 && dy != 0) {
		jx = _jump(gd, (x + dx), y, x, y, endNode);
		jy = _jump(gd, x, (y + dy), x, y, endNode);

		if (jx || jy) {
			int *i;

			if (jx) {
				free(jx);
			}
			if (jy) {
				free(jy);
			}

			i = (int *) malloc(2 * sizeof(int));
			i[0] = x;
			i[1] = y;
			return i;
		}
	}

	if (istraversableAt(gd, (x + dx), y) || istraversableAt(gd, x, (y + dy))) {
		return _jump(gd, (x + dx), (y + dy), x, y, endNode);
	}
	else {
		return NULL;
	}
}

/*
 * Identify successors to the current node, important for knowing where the jump was made from
 */
void _get_successors(struct grid *gd, struct node *activeNode, struct open_list *current, struct node *endNode)
{
	int endX = endNode -> x;
	int endY = endNode -> y;
	int *jumpPoint;
	struct neighbor_xy_list *neighbors_head = _findNeighbors(gd, activeNode);
	struct neighbor_xy_list *neighbors_current = neighbors_head;
	while (neighbors_head != (neighbors_current = neighbors_current -> right)) {

		jumpPoint = _jump(gd, neighbors_current -> x, neighbors_current -> y, activeNode -> x, activeNode -> y, endNode);
		if (jumpPoint != NULL) {
			int jx, jy, d, ng;
			struct node *jumpNode;
			jx = jumpPoint[0];
			jy = jumpPoint[1];

			free(jumpPoint);

			jumpNode = getNodeAt(gd, jx, jy);
			if (jumpNode -> closed) {
				continue;
			}

			d = euclidean(abs(jx - activeNode -> x), abs(jy - activeNode -> y));
			ng = activeNode -> g + d;
			if (!jumpNode -> opened || ng < jumpNode -> g) {
				jumpNode -> g = ng;
				if (!jumpNode -> h)
					jumpNode -> h = manhattan(abs(jx - endX), abs(jy - endY));
				/* jumpNode -> h = jumpNode -> h || manhattan(abs(jx - endX), abs(jy - endY)); // ASK FIDELIS !! */
				jumpNode -> f = jumpNode -> g + jumpNode -> h;
				jumpNode -> parent = activeNode;

				if (!jumpNode -> opened) {
					current = open_list_insert_right(current, jumpNode);
					jumpNode -> opened = true;
				}
				else {
					open_list_sort(current -> right);
				}
			}
		}
	}
	neighbor_xy_clean(neighbors_head);
}

struct neighbor_xy_list *backtrace(struct node *activeNode)
{
	struct neighbor_xy_list *head = neighbor_xy_new();
	struct neighbor_xy_list *current = head;
	current = neighbor_xy_insert_right(current, activeNode -> x, activeNode -> y);
	while (activeNode -> parent != NULL) {
		activeNode = activeNode -> parent;
		current = neighbor_xy_insert_right(current, activeNode -> x, activeNode -> y);
	}
	return head;
}

struct neighbor_xy_list *findPath(struct grid *gd, int startX, int startY, int endX, int endY)
{
	struct open_list *head = open_list_new();
	struct open_list *current = head;
	struct node *startNode = getNodeAt(gd, startX, startY);
	struct node *endNode = getNodeAt(gd, endX, endY);
	struct node *activeNode;
	int counter = 0;

	/* Initialize the start node */
	startNode -> h = 0;
	startNode -> g = 0;
	startNode -> f = 0;
	startNode -> parent = NULL;

	current = open_list_insert_right(current, startNode);

	startNode -> opened = true;

	head = open_list_sort(head);
	current = head -> left;

	while (head != current) {
		activeNode = current -> list_node;
		current = open_list_delete(current);
		activeNode -> closed = true;

		if (activeNode == endNode) {
			struct neighbor_xy_list *goal;
			open_list_clean(head);
			goal = backtrace(activeNode);
			return goal;
		}

		_get_successors(gd, activeNode, current, endNode);
		head = open_list_sort(head);
		current = head -> right;

		counter++;
		if (counter >= 5000) {
			open_list_clean(head);
			printf("\nError: Too many successors\n");
			return NULL;
		}
	}
	printf("\nHead is the current node\n");
	return NULL;
}

/*
 *
 */
struct neighbor_xy_list *smooth_path(struct grid *gd, struct neighbor_xy_list *head)
{
	struct neighbor_xy_list *pos = head -> left;
	int xi, yi, dx, dy;
	while (head != NULL && (head != (pos = pos -> left))) {
		xi = pos -> x;
		yi = pos -> y;
		dx = xi - pos -> right -> x;
		dy = yi - pos -> right -> y;
		if (dx == 1 && dy == -1) {
			if (!istraversableAt(gd, xi, yi + 1)) {
				pos = neighbor_xy_insert_right(pos, pos -> x - 1, pos -> y);
			} else if (!istraversableAt(gd, xi - 1, yi)) {
				pos = neighbor_xy_insert_right(pos, pos -> x, pos -> y + 1);
			}
		} else if (dx == 1 && dy == 1) {
			if (!istraversableAt(gd, xi - 1, yi)) {
				pos = neighbor_xy_insert_right(pos, pos -> x, pos -> y - 1);
			} else if (!istraversableAt(gd, xi, yi - 1)) {
				pos = neighbor_xy_insert_right(pos, pos -> x - 1, pos -> y);
			}
		} else if (dx == -1 && dy == 1) {
			if (!istraversableAt(gd, xi, yi - 1)) {
				pos = neighbor_xy_insert_right(pos, pos -> x + 1, pos -> y);
			} else if (!istraversableAt(gd, xi + 1, yi)) {
				pos = neighbor_xy_insert_right(pos, pos -> x, pos -> y - 1);
			}
		} else if (dx == -1 && dy == -1) {
			if (!istraversableAt(gd, xi + 1, yi)) {
				pos = neighbor_xy_insert_right(pos, pos -> x, pos -> y + 1);
			} else if (!istraversableAt(gd, xi, yi + 1)) {
				pos = neighbor_xy_insert_right(pos, pos -> x + 1, pos -> y);
			}
		} else if (abs(dx) > 1 || abs(dy) > 1) {
			int incrX = dx / max(abs(dx), 1);
			int incrY = dy / max(abs(dy), 1);
			pos = neighbor_xy_insert_right(pos, pos -> right -> x + incrX, pos -> right -> y + incrY);
		}
	}
	return head;
}

struct neighbor_xy_list *neighbor_xy_new()
{
	struct neighbor_xy_list *newlist = (struct neighbor_xy_list *) malloc(sizeof(struct neighbor_xy_list));
	newlist -> right = newlist;
	newlist -> left = newlist;
	newlist -> x = 0;
	newlist -> y = 0;
	return newlist;
}

void neighbor_xy_clean(struct neighbor_xy_list *head)
{
	if (head != NULL) {
		struct neighbor_xy_list *pos = head;
		struct neighbor_xy_list *tmp = head;
		do {
			tmp = pos -> right;
			free(pos);
			pos = tmp;
		} while (pos != head);
	}
}

struct neighbor_xy_list *neighbor_xy_insert_right(struct neighbor_xy_list *list, int x, int y)
{
	struct neighbor_xy_list *newlist = (struct neighbor_xy_list *) malloc(sizeof(struct neighbor_xy_list));
	newlist -> x = x;
	newlist -> y = y;
	newlist -> left = list;
	newlist -> right = list -> right;
	list -> right = newlist;
	newlist -> right -> left = newlist;
	return newlist;
}

struct neighbor_xy_list *_findNeighbors(struct grid *gd, struct node *activeNode)
{
	struct node *parent = activeNode -> parent;
	int x = activeNode -> x;
	int y = activeNode -> y;
	int px, py, dx, dy;

	struct neighbor_xy_list *head = neighbor_xy_new();
	struct neighbor_xy_list *current = head;

	struct node *neighborNode;
	struct neighbor_list *neighborNodes_head;
	struct neighbor_list *neighborNodes_current;

	if (parent) {
		px = parent -> x;
		py = parent -> y;

		dx = (x - px) / max(abs(x - px), 1);
		dy = (y - py) / max(abs(y - py), 1);

		/* Diagonals */
		if (dx != 0 && dy != 0) {
			if (istraversableAt(gd, x, (y + dy))) {
				current = neighbor_xy_insert_right(current, x, (y + dy));
			}
			if (istraversableAt(gd, (x + dx), y)) {
				current = neighbor_xy_insert_right(current, (x + dx), y);
			}
			if (istraversableAt(gd, x, (y + dy)) || istraversableAt(gd, (x + dx), y)) {
				current = neighbor_xy_insert_right(current, (x + dx), (y + dy));
			}
			if (!istraversableAt(gd, (x - dx), y) && istraversableAt(gd, x, (y + dy))) {
				current = neighbor_xy_insert_right(current, (x - dx), (y + dy));
			}
			if (!istraversableAt(gd, x, (y - dy)) && istraversableAt(gd, (x + dx), y)) {
				current = neighbor_xy_insert_right(current, (x + dx), (y - dy));
			}

			/* Horizontal / Vertical */
		} else {
			if (dx == 0) {
				if (istraversableAt(gd, x, (y + dy))) {
					if (istraversableAt(gd, x, (y + dy))) {
						current = neighbor_xy_insert_right(current, x, (y + dy));
					}
					if (!istraversableAt(gd, (x + 1), y)) {
						current = neighbor_xy_insert_right(current, (x + 1), (y + dy));
					}
					if (!istraversableAt(gd, (x - 1), y)) {
						current = neighbor_xy_insert_right(current, (x - 1), (y + dy));
					}
				}
			} else {
				if (istraversableAt(gd, (x + dx), y)) {
					if (istraversableAt(gd, (x + dx), y)) {
						current = neighbor_xy_insert_right(current, (x + dx), y);
					}
					if (!istraversableAt(gd, x, (y + 1))) {
						current = neighbor_xy_insert_right(current, (x + dx), (y + 1));
					}
					if (!istraversableAt(gd, x, (y - 1))) {
						current = neighbor_xy_insert_right(current, (x + dx), (y - 1));
					}
				}
			}
		}
	} else {
		neighborNodes_head = getNeighbors(gd, activeNode);
		neighborNodes_current = neighborNodes_head;
		while (neighborNodes_head != (neighborNodes_current = neighborNodes_current -> right)) {
			neighborNode = neighborNodes_current -> neighbor_node;
			current = neighbor_xy_insert_right(current, neighborNode -> x, neighborNode -> y);
		}
		clean_neighbor_list(neighborNodes_head);
	}

	return head;
}

struct neighbor_list *new_neighbor_list()
{
	struct neighbor_list *newlist = (struct neighbor_list *) malloc(sizeof(struct neighbor_list));
	newlist -> right = newlist;
	newlist -> left = newlist;
	newlist -> neighbor_node = NULL;
	return newlist;
}

void clean_neighbor_list(struct neighbor_list *head)
{
	if (head != NULL) {
		struct neighbor_list *pos = head;
		struct neighbor_list *tmp = head;
		do {
			tmp = pos -> right;
			free(pos);
			pos = tmp;
		} while (pos != head);
	}
}

struct neighbor_list *insert_right(struct neighbor_list *list, struct node *data)
{
	struct neighbor_list *newlist = (struct neighbor_list *) malloc(sizeof(struct neighbor_list));
	newlist -> neighbor_node = data;
	newlist -> left = list;
	newlist -> right = list -> right;
	list -> right = newlist;
	newlist -> right -> left = newlist;
	return newlist;
}

struct node createNode( int x, int y, bool traversable)
{
	struct node nd;
	nd.x = x;
	nd.y = y;
	nd.f = 0;
	nd.g = 0;
	nd.h = 0;
	nd.opened = false;
	nd.closed = false;
	nd.parent = NULL;
	nd.traversable = traversable ? true : false;
	return nd;
}

struct grid createGrid( int width, int height, bool **matrix)
{
	struct grid gd;
	gd.width = width;
	gd.height = height;
	gd.nodes = _buildNodes(width, height, matrix);
	return gd;
}

struct node **_buildNodes( int width, int height, bool **matrix)
{
	int i, j;
	struct node **nodes;
	nodes = (struct node **) malloc(height * sizeof(struct node *));

	for (i = 0; i < height; i++) {
		nodes[i] = (struct node *) malloc(width * sizeof(struct node));
		for (j = 0; j < width; ++j) {
			nodes[i][j] = createNode(j, i, matrix[i][j]);
		}
	}
	return nodes;
}

struct node *getNodeAt(struct grid *gd, int x, int y)
{
	return &gd -> nodes[y][x];
}

bool istraversableAt(struct grid *gd, int x, int y)
{
	return isInside(gd, x, y) && gd -> nodes[y][x].traversable;
}

bool isInside(struct grid *gd, int x, int y)
{
	return (x >= 0 && x < gd -> width) && (y >= 0 && y < gd -> height);
}

void settraversableAt(struct grid *gd, int x, int y, bool traversable)
{
	gd -> nodes[y][x].traversable = traversable;
}

/*
 * Get a list of all 8 neighbors of current node
 */
struct neighbor_list *getNeighbors(struct grid *gd, struct node *nd)
{
	int x = nd -> x;
	int y = nd -> y;

	struct neighbor_list *head = new_neighbor_list();
	struct neighbor_list *current = head;

	bool d0 = false;
	bool d1 = false;
	bool d2 = false;
	bool d3 = false;

	if (istraversableAt(gd, x, y - 1)) {
		current = insert_right(current, &gd -> nodes[y - 1][x]);
		d0 = d1 = true;
	}

	if (istraversableAt(gd, x + 1, y)) {
		current = insert_right(current, &gd -> nodes[y][x + 1]);
		d1 = d2 = true;
	}

	if (istraversableAt(gd, x, y + 1)) {
		current = insert_right(current, &gd -> nodes[y + 1][x]);
		d2 = d3 = true;
	}

	if (istraversableAt(gd, x - 1, y)) {
		current = insert_right(current, &gd -> nodes[y][x - 1]);
		d3 = d0 = true;
	}

	if (d0 && istraversableAt(gd, x - 1, y - 1)) {
		current = insert_right(current, &gd -> nodes[y - 1][x - 1]);
	}

	if (d1 && istraversableAt(gd, x + 1, y - 1)) {
		current = insert_right(current, &gd -> nodes[y - 1][x + 1]);
	}

	if (d2 && istraversableAt(gd, x + 1, y + 1)) {
		current = insert_right(current, &gd -> nodes[y + 1][x + 1]);
	}

	if (d3 && istraversableAt(gd, x - 1, y + 1)) {
		current = insert_right(current, &gd -> nodes[y + 1][x - 1]);
	}

	return head;
}


int cmp(struct open_list *one, struct open_list *two)
{
	int a, b;

	if (one -> list_node == NULL)
		a = 0;
	else
		a = one -> list_node -> f;

	if (two -> list_node == NULL)
		b = 0;
	else
		b = two -> list_node -> f;

	return a - b;
}

struct open_list *open_list_new()
{
	struct open_list *newlist = (struct open_list *) malloc(sizeof(struct open_list));
	newlist -> right = newlist;
	newlist -> left = newlist;
	newlist -> list_node = NULL;
	return newlist;
}

void open_list_clean(struct open_list *head)
{
	if (head != NULL) {
		struct open_list *pos = head;
		struct open_list *tmp = head;
		do {
			tmp = pos -> right;
			free(pos);
			pos = tmp;
		} while (pos != head);
	}
}

struct open_list *open_list_insert_right(struct open_list *list, struct node *data)
{
	struct open_list *newlist = (struct open_list *) malloc(sizeof(struct open_list));
	newlist -> list_node = data;
	newlist -> left = list;
	newlist -> right = list -> right;
	list -> right = newlist;
	newlist -> right -> left = newlist;
	return newlist;
}

struct open_list *open_list_delete(struct open_list *list)
{
	struct open_list *res = list -> left;
	list -> right -> left = list -> left;
	list -> left -> right = list -> right;
	free(list);
	return res;
}

struct open_list *open_list_sort(struct open_list *list)
{
	struct open_list *p, *q, *e, *tail, *oldhead;
	int insize, nmerges, psize, qsize, i;

	/*
	 * Silly special case: if `list' was passed in as NULL, return
	 * NULL immediately.
	 */
	if (!list)
		return NULL;

	insize = 1;

	while (1) {
		p = list;
		oldhead = list;                /* only used for circular linkage */
		list = NULL;
		tail = NULL;

		nmerges = 0; /* count number of merges we do in this pass */

		while (p) {
			nmerges++; /* there exists a merge to be done */
			/* step `insize' places along from p */
			q = p;
			psize = 0;
			for (i = 0; i < insize; i++) {
				psize++;
				q = (q -> right == oldhead ? NULL : q -> right);
				if (!q) break;
			}

			/* if q hasn't fallen off end, we have two lists to merge */
			qsize = insize;

			/* now we have two lists; merge them */
			while (psize > 0 || (qsize > 0 && q)) {

				/* decide whether next element of merge comes from p or q */
				if (psize == 0) {
					/* p is empty; e must come from q. */
					e = q;
					q = q -> right;
					qsize--;
					if (q == oldhead) q = NULL;
				} else if (qsize == 0 || !q) {
					/* q is empty; e must come from p. */
					e = p;
					p = p -> right;
					psize--;
					if (p == oldhead) p = NULL;
				} else if (cmp(p, q) <= 0) {
					/* First element of p is lower (or same);
					 * e must come from p. */
					e = p;
					p = p -> right;
					psize--;
					if (p == oldhead) p = NULL;
				} else {
					/* First element of q is lower; e must come from q. */
					e = q;
					q = q -> right;
					qsize--;
					if (q == oldhead) q = NULL;
				}

				/* add the next element to the merged list */
				if (tail) {
					tail -> right = e;
				} else {
					list = e;
				}
				/* Maintain reverse pointers in a doubly linked list. */
				e -> left = tail;
				tail = e;
			}

			/* now p has stepped `insize' places along, and q has too */
			p = q;
		}

		tail -> right = list;
		list -> left = tail;

		/* If we have done only one merge, we're finished. */
		if (nmerges <= 1) /* allow for nmerges==0, the empty list case */
			return list;

		/* Otherwise repeat, merging lists twice the size */
		insize *= 2;
	}
	return NULL;
}



void printSolution(struct grid *gd, struct neighbor_xy_list *path)
{
	int i, j;
	bool found = false;
	struct neighbor_xy_list *path_pos = path;
	for (i = 0; i < gd -> height; i++) {
		for (j = 0; j < gd -> width; ++j) {
			if (gd -> nodes[i][j].traversable) {
				while (path != (path_pos = path_pos -> left)) {
					if (path_pos -> y == i && path_pos -> x == j) {
						printf("o");
						found = true;
					}
				}
				if (!found)
					printf(".");
			} else
				printf("#");

			found = false;
		}
		printf("\n");
	}
}

void printGrid(struct grid *gd)
{
	int i, j;
	for (i = 0; i < gd -> height; i++) {
		for (j = 0; j < gd -> width; ++j) {
			if (gd -> nodes[i][j].traversable)
				printf(".");
			else
				printf("#");
		}
		printf("\n");
	}
}

void printNodeInfo(struct node *n)
{
	printf("x: %i ", n -> x);
	printf("\ny: %i ", n -> y);
	printf("\nf: %i ", n -> f);
	if (n -> traversable)
		printf("\ntraversable: yes\n\n");
	else
		printf("\ntraversable: no\n\n");
}

void listNeighbors(struct neighbor_list *list)
{
	struct neighbor_list *head = list;
	struct neighbor_list *current = list;
	while (head != (current = current -> right)) {
		printNodeInfo(current -> neighbor_node);
	}
}

void listOpenList(struct open_list *list)
{
	struct open_list *head = list;
	struct open_list *current = list;
	while (head != (current = current -> right)) {
		printNodeInfo(current -> list_node);
	}
}


void runJumpPointSearch()
{
	int i, width = 100, height = 40, startX = 0, startY = 0, endX = 0, endY = 0; /* Set the size of the map */
	FILE *file;
	char c;
	int n, l, count;
	bool **matrix;
	struct grid newgrid;
	struct neighbor_xy_list *path_head = NULL, *path_pos = NULL;
	clock_t c0, c1;
	double runtime_diff_ms;

	/* Prepare the Matrix of traversable / Not traversable - Dynamic Size */
	matrix = (bool **) malloc(height * sizeof(bool *));
	for (i = 0; i < height; i++) {
		matrix[i] = (bool *)malloc(width * sizeof(bool));
	}

	file = fopen ("mapfile.txt", "r");
	n = l = 0;

	if (file == NULL)
		perror("Error reading file");
	else {
		do {
			c = getc (file);
			if (c == '-')
				matrix[l][n] = true;
			else if (c == 'S') {
				printf("\n\nStart node X:%d / Y:%d\n", n, l);
				startY = l;
				startX = n;
				matrix[l][n] = true;
			} else if (c == 'G') {
				printf("Goal node X:%d / Y:%d\n", n, l);
				endY = l;
				endX = n;
				matrix[l][n] = true;
			} else if (c == 'X') {
				matrix[l][n] = false;
			}
			n++;
			if (n > width) {
				n = 0;
				l++;
			}
		} while (c != EOF);
		fclose(file);
	}

	newgrid = createGrid(width, height, matrix);

	c0 = clock();

	path_head = findPath(&newgrid, startX, startY, endX, endY);
	path_pos = path_head;

	c1 = clock();
	runtime_diff_ms = (c1 - c0) * 1000. / CLOCKS_PER_SEC;

	printf("\nCalculation took %.0fms\n", runtime_diff_ms);



	path_head = smooth_path(&newgrid, path_head);
	path_pos = path_head;

	printf("\n\nWaypoints:\n");
	count = 0;
	while (path_head != NULL && (path_head != (path_pos = path_pos -> left))) {
		printf("Step %d: x:%d y:%d\n", count++, path_pos -> x, path_pos -> y);
	}

	printf("\n\n");
	if (path_head != NULL) {
		printSolution(&newgrid, path_head);
	}

	// free all memory
	neighbor_xy_clean(path_head);

	for (i = 0; i < height; i++) {
		free(newgrid.nodes[i]);
	}

	free(newgrid.nodes);

	for (i = 0; i < height; i++) {
		free(matrix[i]);
	}
	free(matrix);
}

int main()
{
	runJumpPointSearch();
	return 0;
}
