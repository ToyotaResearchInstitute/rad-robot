/*
This file is part of ``kdtree'', a library for working with kd-trees.
Copyright (C) 2007-2011 John Tsiombikas <nuclear@member.fsf.org>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. The name of the author may not be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/
/* single nearest neighbor search written by Tamas Nepusz <tamas@cs.rhul.ac.uk>
 */
/* in_bounds written by Philip Kovac <philip.kovac@iweave.com> */
/* nearest_n by https://github.com/rxlfnng/knn */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(WIN32) || defined(__WIN32__)
#include <malloc.h>
#endif

#include "kdtree.h"

#ifdef USE_LIST_NODE_ALLOCATOR

#ifndef NO_PTHREADS
#include <pthread.h>
#else

#ifndef I_WANT_THREAD_BUGS
#error                                                                         \
    "You are compiling with the fast list node allocator, with pthreads disabled! This WILL break if used from multiple threads."
#endif /* I want thread bugs */

#endif /* pthread support */
#endif /* use list node allocator */

struct kdhyperrect {
  int dim;
  double *min, *max; /* minimum/maximum coords */
};

struct kdnode {
  double *pos;
  int dir;
  void *data;

  struct kdnode *left, *right, *next; /* negative/positive/equal side */
};

struct res_node {
  struct kdnode *item;
  double dist_sq;
  struct res_node *next;
};

struct kdtree {
  int dim;
  struct kdnode *root;
  struct kdhyperrect *rect;

  struct kdnode **node_stack;
  size_t depth, node_stack_max, n;
  int node_stack_head;
  void (*destr)(void *);
};

struct kdres {
  struct kdtree *tree;
  struct res_node *rlist, *riter;
  // head for the nearest_n
  struct res_node head;
  int size;
  int expanded;
};

#define SQ(x) ((x) * (x))

static void clear_rec(struct kdtree *tree, void (*destr)(void *));
static int insert_rec(struct kdnode **node, const double *pos, void *data,
                      int dim);
static int rlist_insert(struct res_node *list, struct kdnode *item,
                        double dist_sq);
static void clear_results(struct kdres *set);

static struct kdhyperrect *hyperrect_create(int dim, const double *min,
                                            const double *max);
static void hyperrect_free(struct kdhyperrect *rect);
static struct kdhyperrect *hyperrect_duplicate(const struct kdhyperrect *rect);
static void hyperrect_extend(struct kdhyperrect *rect, const double *pos);
static double hyperrect_dist_sq(struct kdhyperrect *rect, const double *pos);

#ifdef USE_LIST_NODE_ALLOCATOR
static struct res_node *alloc_resnode(void);
static void free_resnode(struct res_node *);
#else
#define alloc_resnode() malloc(sizeof(struct res_node))
#define free_resnode(n) free(n)
#endif

static inline void node_stack_prep(struct kdtree *tree) {
  if (!tree->node_stack || tree->node_stack_max < tree->depth) {
    if (tree->node_stack)
      free(tree->node_stack);
    tree->node_stack = malloc(sizeof(struct kdnode *) * tree->depth);
    tree->node_stack_max = tree->depth;
    tree->node_stack_head = 0;
  }
}

static inline struct kdnode *node_stack_pop(struct kdtree *tree) {
  if (tree->node_stack_head == 0)
    return NULL;
  else
    return tree->node_stack[--tree->node_stack_head];
}
static inline void node_stack_push(struct kdtree *tree, struct kdnode *node) {
  tree->node_stack[tree->node_stack_head++] = node;
}

struct kdtree *kd_create(int k) {
  struct kdtree *tree;

  if (!(tree = malloc(sizeof *tree))) {
    return 0;
  }

  tree->dim = k;
  tree->root = 0;
  tree->destr = 0;
  tree->rect = 0;
  tree->depth = 0;
  tree->n = 0;
  tree->node_stack = 0;
  tree->node_stack_max = 0;
  tree->node_stack_head = 0;

  return tree;
}

int kd_get_dimension(struct kdtree *tree) {
  if (tree) {
    return tree->dim;
  }
  return 0;
}

size_t kd_get_depth(struct kdtree *tree) {
  if (tree) {
    return tree->depth;
  }
  return 0;
}

// Return number of elements in the tree
size_t kd_get_size(struct kdtree *tree) {
  if (tree) {
    return tree->n;
  }
  return 0;
}

void kd_free(struct kdtree *tree) {
  if (tree) {
    kd_clear(tree);
    free(tree);
  }
}

static void clear_rec(struct kdtree *tree, void (*destr)(void *)) {
  struct kdnode *node;

  node_stack_prep(tree);

  while ((node = node_stack_pop(tree))) {
    if (node->left)
      node_stack_push(tree, node->left);
    if (node->next)
      node_stack_push(tree, node->next);
    if (node->right)
      node_stack_push(tree, node->right);

    if (destr) {
      destr(node->data);
    };
    free(node->pos);
    free(node);
  }
}

void kd_clear(struct kdtree *tree) {
  clear_rec(tree, tree->destr);
  tree->root = 0;
  tree->depth = 0;
  tree->n = 0;

  if (tree->node_stack) {
    free(tree->node_stack);
    tree->node_stack = 0;
    tree->node_stack_head = tree->node_stack_max = 0;
  }

  if (tree->rect) {
    hyperrect_free(tree->rect);
    tree->rect = 0;
  }
}

void kd_data_destructor(struct kdtree *tree, void (*destr)(void *)) {
  tree->destr = destr;
}

/*
static int insert_rec(struct kdnode **nptr, const double *pos, void *data, int
dir, int dim)
{
  int new_dir;
  struct kdnode *node;

  if(!*nptr) {
    if(!(node = malloc(sizeof *node))) {
      return -1;
    }
    if(!(node->pos = malloc(dim * sizeof *node->pos))) {
      free(node);
      return -1;
    }
    memcpy(node->pos, pos, dim * sizeof *node->pos);
    node->data = data;
    node->dir = dir;
    node->left = node->right = 0;
    *nptr = node;
    return 0;
  }

  node = *nptr;
  new_dir = (node->dir + 1) % dim;
  if(pos[node->dir] < node->pos[node->dir]) {
    return insert_rec(&(*nptr)->left, pos, data, new_dir, dim);
  }
  return insert_rec(&(*nptr)->right, pos, data, new_dir, dim);
}
*/

static int insert_rec(struct kdnode **nptr, const double *pos, void *data,
                      int dim) {
  // fprintf(stderr, "insert_rec\n");
  struct kdnode *node;
  size_t depth = 0;
  int pos_idx, dir, new_dir = 0;
  while (*nptr) {
    // fprintf(stderr, "nptr %p\n", nptr);
    depth++;
    node = *nptr;
    dir = node->dir;
    new_dir = (dir + 1) % dim;

    if (pos[dir] < node->pos[dir]) {
      nptr = &(node->left);
      continue;
    } else if (pos[dir] > node->pos[dir]) {
      nptr = &(node->right);
      continue;
    } else { // if (pos[dir] == node->pos[dir]) {
      // fprintf(stderr, "Equal\n");
      for (pos_idx = 0; pos_idx < dim; pos_idx++) {
        if (pos[pos_idx] != node->pos[pos_idx]) {
          break;
        }
      }
      if (pos_idx == dim) {
        while (node->next) {
          node = node->next;
        }
        nptr = &(node->next);
        break;
      } else {
        // fprintf(stderr, "in for\n");
        // TODO: Must do something to nptr!
        nptr = &(node->right);
      }
    }
  }

  if (!(node = malloc(sizeof *node))) {
    return -1;
  }

  if (!(node->pos = malloc(dim * sizeof *node->pos))) {
    free(node);
    return -1;
  }

  memcpy(node->pos, pos, dim * sizeof *node->pos);

  node->data = data;
  node->dir = new_dir;
  node->left = node->right = node->next = 0;
  *nptr = node;

  return depth + 1;
}

int kd_insert(struct kdtree *tree, const double *pos, void *data) {
  int result;

  result = insert_rec(&tree->root, pos, data, tree->dim);

  if (result == -1) {
    return -1;
  } else {
    if (result > tree->depth)
      tree->depth = result;
  }

  if (tree->rect == 0) {
    tree->rect = hyperrect_create(tree->dim, pos, pos);
  } else {
    hyperrect_extend(tree->rect, pos);
  }

  tree->n++;

  return 0;
}

int kd_insertf(struct kdtree *tree, const float *pos, void *data) {
  static double sbuf[16];
  double *bptr, *buf = 0;
  int res, dim = tree->dim;

  if (dim > 16) {
#ifndef NO_ALLOCA
    if (dim <= 256)
      bptr = buf = alloca(dim * sizeof *bptr);
    else
#endif
        if (!(bptr = buf = malloc(dim * sizeof *bptr))) {
      return -1;
    }
  } else {
    bptr = buf = sbuf;
  }

  while (dim-- > 0) {
    *bptr++ = *pos++;
  }

  res = kd_insert(tree, buf, data);
#ifndef NO_ALLOCA
  if (tree->dim > 256)
#else
  if (tree->dim > 16)
#endif
    free(buf);
  return res;
}

int kd_insert3(struct kdtree *tree, double x, double y, double z, void *data) {
  double buf[3];
  buf[0] = x;
  buf[1] = y;
  buf[2] = z;
  return kd_insert(tree, buf, data);
}

int kd_insert3f(struct kdtree *tree, float x, float y, float z, void *data) {
  double buf[3];
  buf[0] = x;
  buf[1] = y;
  buf[2] = z;
  return kd_insert(tree, buf, data);
}

static int in_bounds(struct kdtree *tree, const double *min_pos,
                     const double *max_pos, struct res_node *list, int ordered,
                     int dim, int inclusive) {
  int i, is_in_bounds, added_res = 0;
  struct kdnode *node;

  node_stack_prep(tree);
  node_stack_push(tree, tree->root);

  while ((node = node_stack_pop(tree))) {
    is_in_bounds = 1;

    if (inclusive) {
      for (i = 0; i < dim; i++)
        if (node->pos[i] < min_pos[i] || node->pos[i] > max_pos[i]) {
          is_in_bounds = 0;
          break;
        }
    } else {
      for (i = 0; i < dim; i++)
        if (node->pos[i] <= min_pos[i] || node->pos[i] >= max_pos[i]) {
          is_in_bounds = 0;
          break;
        }
    }

    if (is_in_bounds) {
      if (rlist_insert(list, node, -1.0) == -1) {
        return -1;
      }
      added_res++;
    }

    if (!(min_pos[node->dir] > node->pos[node->dir]) && node->left)
      node_stack_push(tree, node->left);

    if (!(max_pos[node->dir] < node->pos[node->dir]) && node->right)
      node_stack_push(tree, node->right);
  }

  return added_res;
}

static int find_nearest(struct kdnode *node, const double *pos, double range,
                        struct res_node *list, int ordered, int dim) {
  double dist_sq, dx;
  int i, ret, added_res = 0;

  if (!node)
    return 0;

  dist_sq = 0;
  for (i = 0; i < dim; i++) {
    dist_sq += SQ(node->pos[i] - pos[i]);
  }
  if (dist_sq <= SQ(range)) {
    if (rlist_insert(list, node, ordered ? dist_sq : -1.0) == -1) {
      return -1;
    }
    added_res = 1;
  }

  dx = pos[node->dir] - node->pos[node->dir];

  ret = find_nearest(dx <= 0.0 ? node->left : node->right, pos, range, list,
                     ordered, dim);
  if (ret >= 0 && fabs(dx) < range) {
    added_res += ret;
    ret = find_nearest(dx <= 0.0 ? node->right : node->left, pos, range, list,
                       ordered, dim);
  }
  if (ret == -1) {
    return -1;
  }
  added_res += ret;

  return added_res;
}

#if 0
static int find_nearest_n(struct kdnode *node, const double *pos, double range, int num, struct rheap *heap, int dim)
{
	double dist_sq, dx;
	int i, ret, added_res = 0;

	if(!node) return 0;

	/* if the photon is close enough, add it to the result heap */
	dist_sq = 0;
	for(i=0; i<dim; i++) {
		dist_sq += SQ(node->pos[i] - pos[i]);
	}
	if(dist_sq <= range_sq) {
		if(heap->size >= num) {
			/* get furthest element */
			struct res_node *maxelem = rheap_get_max(heap);

			/* and check if the new one is closer than that */
			if(maxelem->dist_sq > dist_sq) {
				rheap_remove_max(heap);

				if(rheap_insert(heap, node, dist_sq) == -1) {
					return -1;
				}
				added_res = 1;

				range_sq = dist_sq;
			}
		} else {
			if(rheap_insert(heap, node, dist_sq) == -1) {
				return =1;
			}
			added_res = 1;
		}
	}


	/* find signed distance from the splitting plane */
	dx = pos[node->dir] - node->pos[node->dir];

	ret = find_nearest_n(dx <= 0.0 ? node->left : node->right, pos, range, num, heap, dim);
	if(ret >= 0 && fabs(dx) < range) {
		added_res += ret;
		ret = find_nearest_n(dx <= 0.0 ? node->right : node->left, pos, range, num, heap, dim);
	}

}
#endif

/// Begin https://github.com/rxlfnng/knn
static struct res_node *res_get_max(struct kdres *list) {

  struct res_node *head = &list->head;
  while (head->next) {
    head = head->next;
  }
  return head;
}
void res_remove_max(struct kdres *list) {
  struct res_node *head = &(list->head);
  while (head->next) {
    if (head->next->next) {
      head = head->next;
    } else {
      free_resnode(head->next);
      head->next = NULL;
      list->size--;
    }
  }
}
static int res_insert(struct kdres *list, struct kdnode *item, double dist_sq) {
  struct res_node *rnode;
  struct res_node *head = &list->head;
  if (!(rnode = alloc_resnode())) {
    return -1;
  }
  rnode->item = item;
  rnode->dist_sq = dist_sq;
  if (dist_sq >= 0.0) {
    while (head->next && head->next->dist_sq < dist_sq) {
      head = head->next;
    }
  }
  if (!(head->next)) {
    rnode->next = NULL;
  } else {
    rnode->next = head->next;
  }
  head->next = rnode;
  list->size++;
  return 1;
}
static bool kd_n_nearest_i(struct kdnode *node, const double *pos,
                           struct kdres **result, struct kdhyperrect *rect,
                           int num) {
  int dir = node->dir;
  int i;
  double dummy, dist_sq;
  struct kdnode *nearer_subtree, *farther_subtree;
  double *nearer_hyperrect_coord, *farther_hyperrect_coord;
  /* Check the distance of the point at the current node, compare it
   * with our best so far */
  dist_sq = 0;
  for (i = 0; i < rect->dim; i++) {
    dist_sq += SQ(node->pos[i] - pos[i]);
  }
  if ((*result)->size >= num) {
    /* get furthest element */
    struct res_node *maxelem = res_get_max(*result);

    /* and check if the new one is closer than that */
    if (maxelem->dist_sq > dist_sq) {
      res_remove_max(*result);

      if (res_insert(*result, node, dist_sq) == -1) {
        return false;
      }
    }
  } else {
    if (res_insert(*result, node, dist_sq) == -1) {
      return true;
    }
  }
  /* Decide whether to go left or right in the tree */
  dummy = pos[dir] - node->pos[dir];
  if (dummy <= 0) {
    nearer_subtree = node->left;
    farther_subtree = node->right;
    nearer_hyperrect_coord = rect->max + dir;
    farther_hyperrect_coord = rect->min + dir;
  } else {
    nearer_subtree = node->right;
    farther_subtree = node->left;
    nearer_hyperrect_coord = rect->min + dir;
    farther_hyperrect_coord = rect->max + dir;
  }

  if (nearer_subtree) {
    /* Slice the hyperrect to get the hyperrect of the nearer subtree */
    dummy = *nearer_hyperrect_coord;
    *nearer_hyperrect_coord = node->pos[dir];
    /* Recurse down into nearer subtree */
    kd_n_nearest_i(nearer_subtree, pos, result, rect, num);
    /* Undo the slice */
    *nearer_hyperrect_coord = dummy;
  }

  if (farther_subtree) {
    /* Get the hyperrect of the farther subtree */
    dummy = *farther_hyperrect_coord;
    *farther_hyperrect_coord = node->pos[dir];
    /* Check if we have to recurse down by calculating the closest
     * point of the hyperrect and see if it's closer than our
     * minimum distance in result_dist_sq. */
    if (hyperrect_dist_sq(rect, pos) < res_get_max(*result)->dist_sq) {
      /* Recurse down into farther subtree */
      kd_n_nearest_i(farther_subtree, pos, result, rect, num);
    }
    /* Undo the slice on the hyperrect */
    *farther_hyperrect_coord = dummy;
  }
  // TODO: Is this the correct return type?
  return true;
}
struct kdres *kd_nearest_n(struct kdtree *kd, const double *pos, int num) {
  struct kdhyperrect *rect;
  struct kdres *rset;
  /* Allocate result set */
  if (!(rset = malloc(sizeof *rset))) {
    return 0;
  }
  if (!(rset->rlist = alloc_resnode())) {
    free(rset);
    return 0;
  }
  rset->rlist->next = 0;
  rset->tree = kd;

  /* Duplicate the bounding hyperrectangle, we will work on the copy */
  if (!(rect = hyperrect_duplicate(kd->rect))) {
    kd_res_free(rset);
    return 0;
  }
  rset->size = 0;
  rset->head.next = NULL;
  /* Search for the nearest neighbour recursively */
  kd_n_nearest_i(kd->root, pos, &rset, rect, num);

  /* Free the copy of the hyperrect */
  hyperrect_free(rect);
  return rset;
}
/// End https://github.com/rxlfnng/knn

static void kd_nearest_i(struct kdnode *node, const double *pos,
                         struct kdnode **result, double *result_dist_sq,
                         struct kdhyperrect *rect) {
  int dir = node->dir;
  int i;
  double dummy, dist_sq;
  struct kdnode *nearer_subtree, *farther_subtree;
  double *nearer_hyperrect_coord, *farther_hyperrect_coord;

  /* Decide whether to go left or right in the tree */
  dummy = pos[dir] - node->pos[dir];
  if (dummy <= 0) {
    nearer_subtree = node->left;
    farther_subtree = node->right;
    nearer_hyperrect_coord = rect->max + dir;
    farther_hyperrect_coord = rect->min + dir;
  } else {
    nearer_subtree = node->right;
    farther_subtree = node->left;
    nearer_hyperrect_coord = rect->min + dir;
    farther_hyperrect_coord = rect->max + dir;
  }

  if (nearer_subtree) {
    /* Slice the hyperrect to get the hyperrect of the nearer subtree */
    dummy = *nearer_hyperrect_coord;
    *nearer_hyperrect_coord = node->pos[dir];
    /* Recurse down into nearer subtree */
    kd_nearest_i(nearer_subtree, pos, result, result_dist_sq, rect);
    /* Undo the slice */
    *nearer_hyperrect_coord = dummy;
  }

  /* Check the distance of the point at the current node, compare it
   * with our best so far */
  dist_sq = 0;
  for (i = 0; i < rect->dim; i++) {
    dist_sq += SQ(node->pos[i] - pos[i]);
  }
  if (dist_sq < *result_dist_sq) {
    *result = node;
    *result_dist_sq = dist_sq;
  }

  if (farther_subtree) {
    /* Get the hyperrect of the farther subtree */
    dummy = *farther_hyperrect_coord;
    *farther_hyperrect_coord = node->pos[dir];
    /* Check if we have to recurse down by calculating the closest
     * point of the hyperrect and see if it's closer than our
     * minimum distance in result_dist_sq. */
    if (hyperrect_dist_sq(rect, pos) < *result_dist_sq) {
      /* Recurse down into farther subtree */
      kd_nearest_i(farther_subtree, pos, result, result_dist_sq, rect);
    }
    /* Undo the slice on the hyperrect */
    *farther_hyperrect_coord = dummy;
  }
}

struct kdres *kd_nearest(struct kdtree *kd, const double *pos) {
  struct kdhyperrect *rect;
  struct kdnode *result;
  struct kdres *rset;
  double dist_sq;
  int i;

  if (!kd)
    return 0;
  if (!kd->rect)
    return 0;

  /* Allocate result set */
  if (!(rset = malloc(sizeof *rset))) {
    return 0;
  }
  if (!(rset->rlist = alloc_resnode())) {
    free(rset);
    return 0;
  }
  rset->rlist->next = 0;
  rset->expanded = 0;
  rset->tree = kd;

  /* Duplicate the bounding hyperrectangle, we will work on the copy */
  if (!(rect = hyperrect_duplicate(kd->rect))) {
    kd_res_free(rset);
    return 0;
  }

  /* Our first guesstimate is the root node */
  result = kd->root;
  dist_sq = 0;
  for (i = 0; i < kd->dim; i++)
    dist_sq += SQ(result->pos[i] - pos[i]);

  /* Search for the nearest neighbour recursively */
  kd_nearest_i(kd->root, pos, &result, &dist_sq, rect);

  /* Free the copy of the hyperrect */
  hyperrect_free(rect);

  /* Store the result */
  if (result) {
    if (rlist_insert(rset->rlist, result, -1.0) == -1) {
      kd_res_free(rset);
      return 0;
    }
    rset->size = 1;
    kd_res_rewind(rset);
    return rset;
  } else {
    kd_res_free(rset);
    return 0;
  }
}

struct kdres *kd_nearestf(struct kdtree *tree, const float *pos) {
  static double sbuf[16];
  double *bptr, *buf = 0;
  int dim = tree->dim;
  struct kdres *res;

  if (dim > 16) {
#ifndef NO_ALLOCA
    if (dim <= 256)
      bptr = buf = alloca(dim * sizeof *bptr);
    else
#endif
        if (!(bptr = buf = malloc(dim * sizeof *bptr))) {
      return 0;
    }
  } else {
    bptr = buf = sbuf;
  }

  while (dim-- > 0) {
    *bptr++ = *pos++;
  }

  res = kd_nearest(tree, buf);
#ifndef NO_ALLOCA
  if (tree->dim > 256)
#else
  if (tree->dim > 16)
#endif
    free(buf);
  return res;
}

struct kdres *kd_nearest3(struct kdtree *tree, double x, double y, double z) {
  double pos[3];
  pos[0] = x;
  pos[1] = y;
  pos[2] = z;
  return kd_nearest(tree, pos);
}

struct kdres *kd_nearest3f(struct kdtree *tree, float x, float y, float z) {
  double pos[3];
  pos[0] = x;
  pos[1] = y;
  pos[2] = z;
  return kd_nearest(tree, pos);
}

/* ---- nearest N search ---- */
/*
static kdres *kd_nearest_n(struct kdtree *kd, const double *pos, int num)
{
        int ret;
        struct kdres *rset;

        if(!(rset = malloc(sizeof *rset))) {
                return 0;
        }
        if(!(rset->rlist = alloc_resnode())) {
                free(rset);
                return 0;
        }
        rset->rlist->next = 0;
        rset->expanded = 0;
        rset->tree = kd;

        if((ret = find_nearest_n(kd->root, pos, range, num, rset->rlist,
kd->dim)) == -1) {
                kd_res_free(rset);
                return 0;
        }
        rset->size = ret;
        kd_res_rewind(rset);
        return rset;
}*/

struct kdres *kd_in_bounds(struct kdtree *kd, const double *min_pos,
                           const double *max_pos, int inclusive) {
  int ret;
  struct kdres *rset;

  if (!(rset = malloc(sizeof *rset))) {
    return NULL;
  }

  if (!(rset->rlist = alloc_resnode())) {
    free(rset);
    return NULL;
  }

  rset->rlist->next = 0;
  rset->expanded = 0;
  rset->tree = kd;

  if ((ret = in_bounds(kd, min_pos, max_pos, rset->rlist, 0, kd->dim,
                       inclusive)) == -1) {
    kd_res_free(rset);
    return NULL;
  }

  rset->size = ret;
  kd_res_rewind(rset);
  return rset;
}

struct kdres *kd_nearest_range(struct kdtree *kd, const double *pos,
                               double range) {
  int ret;
  struct kdres *rset;

  if (!(rset = malloc(sizeof *rset))) {
    return 0;
  }
  if (!(rset->rlist = alloc_resnode())) {
    free(rset);
    return 0;
  }
  rset->rlist->next = 0;
  rset->expanded = 0;
  rset->tree = kd;

  if ((ret = find_nearest(kd->root, pos, range, rset->rlist, 1, kd->dim)) ==
      -1) {
    kd_res_free(rset);
    return 0;
  }
  rset->size = ret;
  kd_res_rewind(rset);
  return rset;
}

struct kdres *kd_nearest_rangef(struct kdtree *kd, const float *pos,
                                float range) {
  static double sbuf[16];
  double *bptr, *buf = 0;
  int dim = kd->dim;
  struct kdres *res;

  if (dim > 16) {
#ifndef NO_ALLOCA
    if (dim <= 256)
      bptr = buf = alloca(dim * sizeof *bptr);
    else
#endif
        if (!(bptr = buf = malloc(dim * sizeof *bptr))) {
      return 0;
    }
  } else {
    bptr = buf = sbuf;
  }

  while (dim-- > 0) {
    *bptr++ = *pos++;
  }

  res = kd_nearest_range(kd, buf, range);
#ifndef NO_ALLOCA
  if (kd->dim > 256)
#else
  if (kd->dim > 16)
#endif
    free(buf);
  return res;
}

struct kdres *kd_nearest_range3(struct kdtree *tree, double x, double y,
                                double z, double range) {
  double buf[3];
  buf[0] = x;
  buf[1] = y;
  buf[2] = z;
  return kd_nearest_range(tree, buf, range);
}

struct kdres *kd_nearest_range3f(struct kdtree *tree, float x, float y, float z,
                                 float range) {
  double buf[3];
  buf[0] = x;
  buf[1] = y;
  buf[2] = z;
  return kd_nearest_range(tree, buf, range);
}

void kd_res_free(struct kdres *rset) {
  clear_results(rset);
  free_resnode(rset->rlist);
  free(rset);
}

int kd_res_size(struct kdres *set) { return (set->size); }

static int res_expand_node(struct res_node **rnode) {
  int size_inc = 0;
  struct kdnode *node;
  node = (*rnode)->item;
  while (node->next) {
    *rnode = ((*rnode)->next = alloc_resnode());

    (*rnode)->item = node->next;
    node = node->next;
    size_inc++;
  }
  return size_inc;
}

static int res_expand(struct kdres *rset) {
  struct res_node *rnode, *next_node;
  if (rset->expanded)
    return 0; /* Has already been expanded, don't do anything */
  rnode = rset->riter;
  while (rnode) {
    next_node = rnode->next;
    rset->size += res_expand_node(&rnode);
    rnode = (rnode->next = next_node);
  }
  rset->expanded = 1;
  return 0;
}

void kd_res_rewind(struct kdres *rset) {
  rset->riter = rset->rlist->next;
  if (!(rset->expanded))
    res_expand(rset);
}

int kd_res_end(struct kdres *rset) { return rset->riter == 0; }

int kd_res_next(struct kdres *rset) {
  rset->riter = rset->riter->next;
  return rset->riter != 0;
}

void *kd_res_item(struct kdres *rset, double *pos) {
  if (rset->riter) {
    if (pos) {
      memcpy(pos, rset->riter->item->pos, rset->tree->dim * sizeof *pos);
    }
    return rset->riter->item->data;
  }
  return 0;
}

void *kd_res_item_dist_sq(struct kdres *rset, double *pos, double *dist_sq) {
  if (rset->riter) {
    if (pos) {
      memcpy(pos, rset->riter->item->pos, rset->tree->dim * sizeof *pos);
    }
    if (dist_sq) {
      *dist_sq = rset->riter->dist_sq;
    }
    return rset->riter->item->data;
  }
  return 0;
}

void *kd_res_itemf(struct kdres *rset, float *pos) {
  if (rset->riter) {
    if (pos) {
      int i;
      for (i = 0; i < rset->tree->dim; i++) {
        pos[i] = rset->riter->item->pos[i];
      }
    }
    return rset->riter->item->data;
  }
  return 0;
}

void *kd_res_item3(struct kdres *rset, double *x, double *y, double *z) {
  if (rset->riter) {
    if (x)
      *x = rset->riter->item->pos[0];
    if (y)
      *y = rset->riter->item->pos[1];
    if (z)
      *z = rset->riter->item->pos[2];
    return rset->riter->item->data;
  }
  return 0;
}

void *kd_res_item3f(struct kdres *rset, float *x, float *y, float *z) {
  if (rset->riter) {
    if (x)
      *x = rset->riter->item->pos[0];
    if (y)
      *y = rset->riter->item->pos[1];
    if (z)
      *z = rset->riter->item->pos[2];
    return rset->riter->item->data;
  }
  return 0;
}

void *kd_res_item_data(struct kdres *set) { return kd_res_item(set, 0); }

/* ---- hyperrectangle helpers ---- */
static struct kdhyperrect *hyperrect_create(int dim, const double *min,
                                            const double *max) {
  size_t size = dim * sizeof(double);
  struct kdhyperrect *rect = 0;

  if (!(rect = malloc(sizeof(struct kdhyperrect)))) {
    return NULL;
  }

  rect->dim = dim;
  if (!(rect->min = malloc(size))) {
    free(rect);
    return NULL;
  }
  if (!(rect->max = malloc(size))) {
    free(rect->min);
    free(rect);
    return NULL;
  }
  memcpy(rect->min, min, size);
  memcpy(rect->max, max, size);

  return rect;
}

static void hyperrect_free(struct kdhyperrect *rect) {
  free(rect->min);
  free(rect->max);
  free(rect);
}

static struct kdhyperrect *hyperrect_duplicate(const struct kdhyperrect *rect) {
  return hyperrect_create(rect->dim, rect->min, rect->max);
}

static void hyperrect_extend(struct kdhyperrect *rect, const double *pos) {
  int i;

  for (i = 0; i < rect->dim; i++) {
    if (pos[i] < rect->min[i]) {
      rect->min[i] = pos[i];
    }
    if (pos[i] > rect->max[i]) {
      rect->max[i] = pos[i];
    }
  }
}

static double hyperrect_dist_sq(struct kdhyperrect *rect, const double *pos) {
  int i;
  double result = 0;

  for (i = 0; i < rect->dim; i++) {
    if (pos[i] < rect->min[i]) {
      result += SQ(rect->min[i] - pos[i]);
    } else if (pos[i] > rect->max[i]) {
      result += SQ(rect->max[i] - pos[i]);
    }
  }

  return result;
}

/* ---- static helpers ---- */

#ifdef USE_LIST_NODE_ALLOCATOR
/* special list node allocators. */
static struct res_node *free_nodes;

#ifndef NO_PTHREADS
static pthread_mutex_t alloc_mutex = PTHREAD_MUTEX_INITIALIZER;
#endif

static struct res_node *alloc_resnode(void) {
  struct res_node *node;

#ifndef NO_PTHREADS
  pthread_mutex_lock(&alloc_mutex);
#endif

  if (!free_nodes) {
    node = malloc(sizeof *node);
  } else {
    node = free_nodes;
    free_nodes = free_nodes->next;
    node->next = 0;
  }

#ifndef NO_PTHREADS
  pthread_mutex_unlock(&alloc_mutex);
#endif

  return node;
}

static void free_resnode(struct res_node *node) {
#ifndef NO_PTHREADS
  pthread_mutex_lock(&alloc_mutex);
#endif

  node->next = free_nodes;
  free_nodes = node;

#ifndef NO_PTHREADS
  pthread_mutex_unlock(&alloc_mutex);
#endif
}
#endif /* list node allocator or not */

/* inserts the item. if dist_sq is >= 0, then do an ordered insert */
/* TODO make the ordering code use heapsort */
static int rlist_insert(struct res_node *list, struct kdnode *item,
                        double dist_sq) {
  struct res_node *rnode;

  if (!(rnode = alloc_resnode())) {
    return -1;
  }
  rnode->item = item;
  rnode->dist_sq = dist_sq;

#ifndef SKIP_SORT
  if (dist_sq >= 0.0) {
    while (list->next && list->next->dist_sq < dist_sq) {
      list = list->next;
    }
  }
#endif
  rnode->next = list->next;
  list->next = rnode;
  return 0;
}

static void clear_results(struct kdres *rset) {
  struct res_node *tmp, *node = rset->rlist->next;

  while (node) {
    tmp = node;
    node = node->next;
    free_resnode(tmp);
  }
  rset->rlist->next = NULL;

  struct res_node *head = rset->head.next;
  while (head) {
    tmp = head;
    head = head->next;
    free_resnode(tmp);
  }
  rset->head.next = NULL;

  rset->expanded = 0;
}
