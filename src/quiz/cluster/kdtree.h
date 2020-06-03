/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

enum class KdBoxState : int {
	BOX_IS_EXCLUSIVLY_LEFT = 0,
	BOX_IS_EXCLUSIVLY_RIGHT = 1,
	BOX_GOT_INTERSECTED = 2
};

// A box for a Kdtree.
// Dim is the dimension (i.e., Dim=2 for the quiz and 3 for the real-world).
// T is either float or double.
template<typename T, int Dim>
struct KdBox {
	// Variable for the dimension.
	int N{Dim};

	// Center of the box (the target point).
	std::vector<T> center;

	// Distance tolerance.
	T distanceTolerance;

	// Minimum values for each dimension
	std::array<T, Dim> lower_bounds;
	// Maximum values for each dimension
	std::array<T, Dim> upper_bounds;

	// ctor
	KdBox(const std::vector<T> target, T distTol) : center(target), distanceTolerance(distTol) {
		for (int d = 0; d < Dim; d++) {
			lower_bounds[d] = center[d] - distanceTolerance;
			upper_bounds[d] = center[d] + distanceTolerance;
		}
	}

	// Check if a given point is within the box or not.
	bool isPointWithinBox(const std::vector<T>& point) const {
		for (int d = 0; d < Dim; d++) {
			if (point[d] < lower_bounds[d]) {
				return false;
			}
			if (point[d] > upper_bounds[d]) {
				return false;
			}
		}
		return true;
	}

	// Compute a point's distance to the box center.
	T distanceToBoxCenter(const std::vector<T> &pt) const {
		T distance = 0.0;
		for (int d = 0; d < Dim; d++) {
			T delta = pt[d] - this->center[d];
			distance += delta*delta;
		}
		return std::sqrt(distance);
	}

	// Check if a point is within the distance tolerance.
	bool isPointWithinDistanceTolerance(const std::vector<T>& pt) const {
		T distance = this->distanceToBoxCenter(pt);
		if (distance > this->distanceTolerance) {
			return false;
		} else {
			return true;
		}
	}

	// Check the position of a box wrt/ a plane which is defined by
	// one constant coordinate.
	// For instance, the plane (or 2D-line) could be defined by x = 5.3.
	// If the box is completely within the space with x < 5.3, then the box
	// is said to be on the left.
	// If it is completely within the space with x > 5.3, then the box
	// is said to be on the right.
	// If neither is the case, the plane (or line) intersects the box.
	//
	// The input to this function is the point under interest along with
	// the dimension. For instance, in the example above, the point could
	// be (5.3, 1.8, -3.1) with current dimension = 0 (for x; 1=y; 2=z).
	KdBoxState getBoxState(const std::vector<T>& point_on_separating_plane, int current_dimension) const {
		T plane_coordinate = point_on_separating_plane[current_dimension];
		if (upper_bounds[current_dimension] < plane_coordinate) {
			return KdBoxState::BOX_IS_EXCLUSIVLY_LEFT;
		} else if (lower_bounds[current_dimension] > plane_coordinate) {
			return KdBoxState::BOX_IS_EXCLUSIVLY_RIGHT;
		} else {
			return KdBoxState::BOX_GOT_INTERSECTED;
		}
	}

	// It turns out that this "exclusivly-left" vs. "exclusivly-right" vs. "intersecting"
	// thinking is not good.
	// It is better (see solution) to just be concerned about whether or not
	// a box is (partly) within one half space.
	// That is all we need to know, as we then know that we need to consider this half space.
	bool isBoxInLeftHalfSpace(const std::vector<T>& point_on_separating_plane, int current_dimension) const {
		T plane_coordinate = point_on_separating_plane[current_dimension];
		if (lower_bounds[current_dimension] > plane_coordinate) {
			return false; 	// The box is exclusivly-right
		} else {
			return true;	// The box might at least be partly in the left half space.
		}
	}

	bool isBoxInRightHalfSpace(const std::vector<T>& point_on_separating_plane, int current_dimension) const {
		T plane_coordinate = point_on_separating_plane[current_dimension];
		if (upper_bounds[current_dimension] < plane_coordinate) {
			return false; 	// The box is exclusivly-left
		} else {
			return true;	// The box might at least be partly in the right half space.
		}
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

		if (root == NULL) {
			root = new Node(point, id);
			return;
		}

		Node *n = root;
		Node *parent = NULL;
		int depth = 0;
		bool attach_to_left = false;
		while (n != NULL) {
			parent = n;
			// Current dimension
			// cd == 0: Split along x
			// cd == 1: Split along y
			int cd = depth % 2;
			if (point[cd] < n->point[cd]) {
				// Take the left branch
				n = n->left;
				attach_to_left = true;
			} else {
				// Take the right branch
				n = n->right;
				attach_to_left = false;
			}

			// Increase the depth
			depth++;
		}
		// Now n is NULL: create a new node and attach it to the parent.
		n = new Node(point, id);
		if (attach_to_left) {
			parent->left = n;
		} else {
			parent->right = n;
		}

	}

	// \param node Node under consideration. Can be NULL.
	void searchHelper(std::vector<int> *ids, int *nodes_touched, const KdBox<float, 2>& box, const Node *node, int depth) {
		if (node == NULL) {
			return;
		}
		std::cout << "Checking node " << node->id << ".\n";
		(*nodes_touched)++;	 // Dereferencing has a lower operator priority than suffix incrementing!
		// std::cout << "*nodes_touched = " << *nodes_touched << ".\n";

		// Easy access to the point.
		const std::vector<float>& point = node->point;

		// First check if the current node is within the box.
		// If so, compute the distance to the box center (i.e., the target),
		// and if it is smaller than the tolerance add it to 'ids'.

		if (box.isPointWithinBox(point)) {
			// Only call this if the point is within the box, as this
			// is cheaper.
			if (box.isPointWithinDistanceTolerance(point)) {
				ids->push_back(node->id);
				std::cout << "Node " << node->id << " is within the tolerance.\n";
				// Earlier, before refactoring: Check both child nodes.
				// This is actually wrong. Just because this point is within the box,
				// it does not mean that the child nodes need to be.
				// That is because they can be somewhere completely different in the space.
				// All we know about them is that they are on one of either two sides
				// of a plane.
			}
		}

		// Now check the state of the box with respect to the separating plane.
		int current_dimension = depth % box.N;
		if (box.isBoxInLeftHalfSpace(point, current_dimension)) {
			searchHelper(ids, nodes_touched, box, node->left, depth+1);
		}
		if (box.isBoxInRightHalfSpace(point, current_dimension)) {
			searchHelper(ids, nodes_touched, box, node->right, depth+1);
		}

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		// Start with the root node.
		Node *node = this->root;

		std::cout << "Looking for target (" << target[0] << ", " << target[1] << ") with tolerance " << distanceTol << ".\n";
		int nodes_touched = 0;

		// Create a box around the target point.
		KdBox<float, 2> box(target, distanceTol);

		searchHelper(&ids, &nodes_touched, box, node, 0);
		std::cout << "Touched " << nodes_touched << " nodes in total.\n";

		return ids;
	}
	

};




