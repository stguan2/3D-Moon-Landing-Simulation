//  Kevin M. Smith - Basic Octree Class - CS134/235 4/18/18
//


#include "Octree.h"
 

// draw Octree (recursively)
//
void Octree::draw(TreeNode & node, int numLevels, int level) {
	if (level >= numLevels) return;

	if (level == 0) {
		ofSetColor(ofColor::white);
	}
	if (level == 1) {
		ofSetColor(ofColor::pink);
	}
	if (level == 2) {
		ofSetColor(ofColor::blue);
	}
	if (level == 3) {
		ofSetColor(ofColor::purple);
	}
	if (level == 4) {
		ofSetColor(ofColor::yellow);
	}
	if (level == 5) {
		ofSetColor(ofColor::green);
	}

	drawBox(node.box);
	level++;

	for (int i = 0; i < node.children.size(); i++) {
		draw(node.children[i], numLevels, level);
	}
}

// draw only leaf Nodes
//
void Octree::drawLeafNodes(TreeNode & node) {
	if (node.children.size() == 0) {
		drawBox(node.box);
	}
	else {
		for (int i = 0; i < node.children.size(); i++) {
			drawLeafNodes(node.children[i]);
		}
	}
	
}


//draw a box from a "Box" class  
//
void Octree::drawBox(const Box &box) {
	Vector3 min = box.parameters[0];
	Vector3 max = box.parameters[1];
	Vector3 size = max - min;
	Vector3 center = size / 2 + min;
	ofVec3f p = ofVec3f(center.x(), center.y(), center.z());
	float w = size.x();
	float h = size.y();
	float d = size.z();
	ofDrawBox(p, w, h, d);
}

// return a Mesh Bounding Box for the entire Mesh
//
Box Octree::meshBounds(const ofMesh & mesh) {
	int n = mesh.getNumVertices();
	ofVec3f v = mesh.getVertex(0);
	ofVec3f max = v;
	ofVec3f min = v;
	for (int i = 1; i < n; i++) {
		ofVec3f v = mesh.getVertex(i);

		if (v.x > max.x) max.x = v.x;
		else if (v.x < min.x) min.x = v.x;

		if (v.y > max.y) max.y = v.y;
		else if (v.y < min.y) min.y = v.y;

		if (v.z > max.z) max.z = v.z;
		else if (v.z < min.z) min.z = v.z;
	}
//	cout << "vertices: " << n << endl;
//	cout << "min: " << min << "max: " << max << endl;
	return Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
}

// getMeshPointsInBox:  return an array of indices to points in mesh that are contained 
//                      inside the Box.  Return count of points found;
//
int Octree::getMeshPointsInBox(const ofMesh & mesh, const vector<int>& points,
	Box & box, vector<int> & pointsRtn)
{
	int count = 0;
	for (int i = 0; i < points.size(); i++) {
		ofVec3f v = mesh.getVertex(points[i]);
		if (box.inside(Vector3(v.x, v.y, v.z))) {
			count++;
			pointsRtn.push_back(points[i]);
		}
	}
	return count;
}



//  Subdivide a Box into eight(8) equal size boxes, return them in boxList;
//
void Octree::subDivideBox8(const Box &box, vector<Box> & boxList) {
	Vector3 min = box.parameters[0];
	Vector3 max = box.parameters[1];
	Vector3 size = max - min;
	Vector3 center = size / 2 + min;
	float xdist = (max.x() - min.x()) / 2;
	float ydist = (max.y() - min.y()) / 2;
	float zdist = (max.z() - min.z()) / 2;
	Vector3 h = Vector3(0, ydist, 0);

	//  generate ground floor
	//
	Box b[8];
	b[0] = Box(min, center);
	b[1] = Box(b[0].min() + Vector3(xdist, 0, 0), b[0].max() + Vector3(xdist, 0, 0));
	b[2] = Box(b[1].min() + Vector3(0, 0, zdist), b[1].max() + Vector3(0, 0, zdist));
	b[3] = Box(b[2].min() + Vector3(-xdist, 0, 0), b[2].max() + Vector3(-xdist, 0, 0));

	boxList.clear();
	for (int i = 0; i < 4; i++)
		boxList.push_back(b[i]);

	// generate second story
	//
	for (int i = 4; i < 8; i++) {
		b[i] = Box(b[i - 4].min() + h, b[i - 4].max() + h);
		boxList.push_back(b[i]);
	}
}

void Octree::create(const ofMesh & geo, int numLevels) {
	root.box = meshBounds(geo);
	for (int i = 0; i < geo.getNumVertices(); i++) {
		root.points.push_back(i);
	}


	subdivide(geo, root, numLevels, 0);

}

void Octree::subdivide(const ofMesh & mesh, TreeNode & node, int numLevels, int level) {
	// base case
	if (level >= numLevels) return;

	// list of child boxes
	vector<Box> childBox;

	// subdivide box into 8
	subDivideBox8(node.box, childBox);

	// create node for each box
	TreeNode childNode[8];
	for (int i = 0; i < childBox.size(); i++) {
		childNode[i].box = childBox[i];
	}

	// iterate through the 8 child nodes
	for (int i = 0; i < 8; i++) {
		// list of point indices for each child node
		vector<int> pnts;
		int count = getMeshPointsInBox(mesh, node.points, childNode[i].box, pnts);
		// if child node has at least one point, add to node.children
		if (count >= 1) {
			for (int j = 0; j < count; j++) {
				childNode[i].points.push_back(pnts[j]);
			}
			node.children.push_back(childNode[i]);
		}
	}

	// if node (parent node) has no children, do not subdivide again
	if (node.children.size() > 0) {
		for (int i = 0; i < node.children.size(); i++) {
			subdivide(mesh, node.children[i], numLevels, level + 1);
		}
	}
}

bool Octree::intersect(const Ray &ray, const TreeNode & node, TreeNode & nodeRtn) {
	if (node.box.intersect(ray, 0, 10000)) {

		//base case
		if (node.children.size() == 0) {
			nodeRtn = node;
			return true;
		}

		//intersect recursively if it has children
		for (int i = 0; i < node.children.size(); i++) {
			intersect(ray, node.children[i], nodeRtn);
		}
	}
	else {
		return false;
	}
}

//Checks if point intersects with octree
//Node is constant but using const creates errors
//node.box.inside(pt) complains and I am unsure why
//
bool Octree::ptIntersect(const ofVec3f &point, TreeNode & node, TreeNode & nodeRtn)
{
	Vector3 pt = Vector3(point.x, point.y, point.z);
	if (node.box.inside(pt)) {

		//base case
		if (node.children.size() == 0) {
			nodeRtn = node;
			return true;
		}

		//ptIntersect recursively if it has children
		for (int i = 0; i < node.children.size(); i++) {
			ptIntersect(point, node.children[i], nodeRtn);
		}
	}
	else {
		return false;
	}
}




