#include "Renderer.h"
#include "RenderObject.h"
#include <math.h>
#include <algorithm>

#pragma comment(lib, "nclgl.lib")

using std::vector;
using std::string;
using std::cout;
using std::endl;

/*Composite type of Edge storing information
like the id of the edge, the position of the
starting node and the position of the end node,
as well as the cost of that edge*/
struct Edge {
	int edge_id;
	struct Node *node_start;
	struct Node *node_end;
	int cost = 1;
	bool passable = true;
};

/*Composite type of Node storing information
like the id of the node, the node's position
and the edges that are connected to that node*/
struct Node {
	int node_id;
	int parent_id;
	Vector3 node_position;
	vector<Edge*> edges;
	float heuristic;
	float f_value;
	float g_value = 0;

	//comparison operator require to sort a vector of nodes by the value of F
	bool operator<(const Node& b) const{
		return f_value < b.f_value;
	}
};

//Array storing all nodes
Node all_nodes[60];

//Array storing all node positions
Vector3 node_positions[60];

//Greek letter phi as used in the document
float phi = (1.0f + sqrt(5.0f)) / 2.0f;

//Initial smallest distance required to calculate the distance between nodes
float smallest_distance = 100.0f;

//Iterator variable needed to create the IDs for the edges
int iter = 0;

float totalPathCost = 0.0f;
vector<int> open_list;
vector<int> closed_list;

//Populate the array of node positions to then use that array when creating the actual nodes
void createNodePositions(){
	node_positions[0] = Vector3(0, 1, 3 * phi);
	node_positions[1] = Vector3(0, 1, -3 * phi);
	node_positions[2] = Vector3(0, -1, 3 * phi);
	node_positions[3] = Vector3(0, -1, -3 * phi);
	node_positions[4] = Vector3(1, 3 * phi, 0);
	node_positions[5] = Vector3(1, -3 * phi, 0);
	node_positions[6] = Vector3(-1, 3 * phi, 0);
	node_positions[7] = Vector3(-1, -3 * phi, 0);
	node_positions[8] = Vector3(3 * phi, 0, 1);
	node_positions[9] = Vector3(3 * phi, 0, -1);
	node_positions[10] = Vector3(-3 * phi, 0, 1);
	node_positions[11] = Vector3(-3 * phi, 0, -1);
	node_positions[12] = Vector3(2, (1 + 2 * phi), phi);
	node_positions[13] = Vector3(2, (1 + 2 * phi), -phi);
	node_positions[14] = Vector3(2, -(1 + 2 * phi), phi);
	node_positions[15] = Vector3(2, -(1 + 2 * phi), -phi);
	node_positions[16] = Vector3(-2, (1 + 2 * phi), phi);
	node_positions[17] = Vector3(-2, (1 + 2 * phi), -phi);
	node_positions[18] = Vector3(-2, -(1 + 2 * phi), phi);
	node_positions[19] = Vector3(-2, -(1 + 2 * phi), -phi);
	node_positions[20] = Vector3((1 + 2 * phi), phi, 2);
	node_positions[21] = Vector3((1 + 2 * phi), phi, -2);
	node_positions[22] = Vector3((1 + 2 * phi), -phi, 2);
	node_positions[23] = Vector3((1 + 2 * phi), -phi, -2);
	node_positions[24] = Vector3(-(1 + 2 * phi), phi, 2);
	node_positions[25] = Vector3(-(1 + 2 * phi), phi, -2);
	node_positions[26] = Vector3(-(1 + 2 * phi), -phi, 2);
	node_positions[27] = Vector3(-(1 + 2 * phi), -phi, -2);
	node_positions[28] = Vector3(phi, 2, (1 + 2 * phi));
	node_positions[29] = Vector3(phi, 2, -(1 + 2 * phi));
	node_positions[30] = Vector3(phi, -2, (1 + 2 * phi));
	node_positions[31] = Vector3(phi, -2, -(1 + 2 * phi));
	node_positions[32] = Vector3(-phi, 2, (1 + 2 * phi));
	node_positions[33] = Vector3(-phi, 2, -(1 + 2 * phi));
	node_positions[34] = Vector3(-phi, -2, (1 + 2 * phi));
	node_positions[35] = Vector3(-phi, -2, -(1 + 2 * phi));
	node_positions[36] = Vector3(1, (2 + phi), 2 * phi);
	node_positions[37] = Vector3(1, (2 + phi), -2 * phi);
	node_positions[38] = Vector3(1, -(2 + phi), 2 * phi);
	node_positions[39] = Vector3(1, -(2 + phi), -2 * phi);
	node_positions[40] = Vector3(-1, (2 + phi), 2 * phi);
	node_positions[41] = Vector3(-1, (2 + phi), -2 * phi);
	node_positions[42] = Vector3(-1, -(2 + phi), 2 * phi);
	node_positions[43] = Vector3(-1, -(2 + phi), -2 * phi);
	node_positions[44] = Vector3((2 + phi), 2 * phi, 1);
	node_positions[45] = Vector3((2 + phi), 2 * phi, -1);
	node_positions[46] = Vector3((2 + phi), -2 * phi, 1);
	node_positions[47] = Vector3((2 + phi), -2 * phi, -1);
	node_positions[48] = Vector3(-(2 + phi), 2 * phi, 1);
	node_positions[49] = Vector3(-(2 + phi), 2 * phi, -1);
	node_positions[50] = Vector3(-(2 + phi), -2 * phi, 1);
	node_positions[51] = Vector3(-(2 + phi), -2 * phi, -1);
	node_positions[52] = Vector3(2 * phi, 1, (2 + phi));
	node_positions[53] = Vector3(2 * phi, 1, -(2 + phi));
	node_positions[54] = Vector3(2 * phi, -1, (2 + phi));
	node_positions[55] = Vector3(2 * phi, -1, -(2 + phi));
	node_positions[56] = Vector3(-2 * phi, 1, (2 + phi));
	node_positions[57] = Vector3(-2 * phi, 1, -(2 + phi));
	node_positions[58] = Vector3(-2 * phi, -1, (2 + phi));
	node_positions[59] = Vector3(-2 * phi, -1, -(2 + phi));
}

//Specify the node IDs and the node positions
void createNodes(){
	for (int i = 0; i < 60; i++){
		all_nodes[i].node_id = i;
		all_nodes[i].node_position = node_positions[i];
	}
}

//From the spheroid diagram it is obvious that the distances between all nodes
//is exactly the same, and in order to find that distance one must find the 
//smallest distance between the nodes. The function below does that.
void findSmallestN2NDistance(){
	for (int i = 0; i < 60; i++){
		for (int j = 0; j < 60; j++){
			if (i != j){
				float a = (node_positions[j].x - node_positions[i].x) * (node_positions[j].x - node_positions[i].x);
				float b = (node_positions[j].y - node_positions[i].y) * (node_positions[j].y - node_positions[i].y);
				float z = (node_positions[j].z - node_positions[i].z) * (node_positions[j].z - node_positions[i].z);
				float distance = sqrt(a + b + z);
				if (distance < smallest_distance){
					smallest_distance = distance;
				}
			}
		}
	}
}

//Find all of the edges for each node and store them within
//each node struct
void findEdges(){
	for (int i = 0; i < 60; i++){
		for (int j = i + 1; j < 60; j++){
			float a = (node_positions[j].x - node_positions[i].x) * (node_positions[j].x - node_positions[i].x);
			float b = (node_positions[j].y - node_positions[i].y) * (node_positions[j].y - node_positions[i].y);
			float z = (node_positions[j].z - node_positions[i].z) * (node_positions[j].z - node_positions[i].z);
			float distance = sqrt(a + b + z);
			if ((distance <= smallest_distance + 0.005f)){
				Edge* temp = new Edge();

				temp->node_start = &all_nodes[i];
				temp->node_end = &all_nodes[j];

				temp->edge_id = iter;

				all_nodes[i].edges.push_back(temp);
				all_nodes[j].edges.push_back(temp);
				iter++;
			}
		}
	}
}

bool findNode(int A){
	bool foundA = false;
	for (int i = 0; i < 60; i++){
		if (A == all_nodes[i].node_id){
			foundA = true;
		}
	}
	if (foundA == true ){
		return true;
	}
	if (foundA == false ){
		return false;
	}
}

//Lists which edges are connected to a given node
void printNodeEdges(int node_id){
	if (findNode(node_id) == true){
		cout << "-----------------------" << endl;
		cout << "Node id:" << " " << node_id << endl;
		cout << "Edges - End Node id:" << endl;
		for (int i = 0; i < 3; i++){
			cout << "E" << all_nodes[node_id].edges[i]->edge_id << " - ";
			if (all_nodes[node_id].edges[i]->node_end->node_id == node_id){
				cout << "N" << all_nodes[node_id].edges[i]->node_start->node_id << " ";
			}
			else{
				cout << "N" << all_nodes[node_id].edges[i]->node_end->node_id << " ";
			}
			cout << "Cost: " << all_nodes[node_id].edges[i]->cost << endl;
		}
		cout << endl << "-----------------------" << endl;
	}
	else{
		cout << "Provided Node was not found!" << endl;
	}
}

//Calculate the heuristic of node A (current node) in relation to node B (destination)
void calculateH(int B){
	for (int i = 0; i < 60; i++){
		float a = (all_nodes[B].node_position.x - all_nodes[i].node_position.x) * (all_nodes[B].node_position.x - all_nodes[i].node_position.x);
		float b = (all_nodes[B].node_position.y - all_nodes[i].node_position.y) * (all_nodes[B].node_position.y - all_nodes[i].node_position.y);
		float z = (all_nodes[B].node_position.z - all_nodes[i].node_position.z) * (all_nodes[B].node_position.z - all_nodes[i].node_position.z);
		float distance = sqrt(a + b + z);
		all_nodes[i].heuristic = distance;
	}
}

//Do specified nodes exist?
bool nodesExist(int A, int B){
	bool foundA = false;
	bool foundB = false;
	for (int i = 0; i < 60; i++){
		if (A == all_nodes[i].node_id){
			foundA = true;
		}
	}
	for (int i = 0; i < 60; i++){
		if (B == all_nodes[i].node_id){
			foundB = true;
		}
	}
	if (foundA == true && foundB == true){
		return true;
	}
	if (foundA == false || foundB == false){
		return false;
	}
}



//Returns the ID a an edge between child node and a parent node
int findN2NEdge(int node_id, int parent_id){
	for (int i = 0; i < all_nodes[node_id].edges.size(); i++){
		if (all_nodes[node_id].edges[i]->node_end->node_id == parent_id || all_nodes[node_id].edges[i]->node_start->node_id == parent_id){
			return all_nodes[node_id].edges[i]->edge_id;
		}
	}
}

//Prints out the path that got calculated
//The path is calculated backwards as it uses the parent ids
//to track back to the starting node
void printNodePath(int dest){
	while (all_nodes[dest].parent_id != NULL){
		cout << "N" << dest << "<-" << "E" << findN2NEdge(dest, all_nodes[dest].parent_id) << "<-";
		dest = all_nodes[dest].parent_id;
	}
	cout << "N" << dest << endl;
	cout << "Total cost of the path: " << totalPathCost << endl;
}

//Returns the ID of a node that is connected to a given node through a specified edge
int findEndNode(int id, int eid){
	for (int i = 0; i < 3; i++){
		if (all_nodes[id].edges.at(i)->edge_id == eid){
			if (all_nodes[id].edges.at(i)->node_start->node_id == id){
				return all_nodes[id].edges.at(i)->node_end->node_id;
			}
			else{
				return all_nodes[id].edges.at(i)->node_start->node_id;
			}
		}
	}
}

//A* Path Finding Algorithm
//Let A = starting point, Let B = goal point.
string aStarAlgorithm(int A, int B){

	int temp_Q = 0;
	int temp_g = 0;
	//Calculate G and H values for A
	//The initial value G for A will be 0
	all_nodes[A].g_value = 0;

	//Calculate the F-value of A
	all_nodes[A].f_value = all_nodes[A].g_value + all_nodes[A].heuristic;

	//Add A to the Open List. At this point, A is the only node on the Open List.
	open_list.push_back(A);

	//If the Open List is empty(P = null), exit.A path cannot be found.
	while (!open_list.empty())
	{
		//If nodes don't exist exit loop and finish
		if (nodesExist(A, B) == false){
			return "Cannot proceed! The specified node(s) does not exist!";
		}
		//If P is the goal node(P = B)
		if (open_list[0] == B){
			//Starting at B (the most recently-added node to the Closed List), track back through the parents of
			//each node until you reach A.This is the optimal path from A to B.
			printNodePath(B);
			return " ";
		}
		for (int i = 0; i < all_nodes[open_list[0]].edges.size(); i++){
			//If edge is passable
			if (all_nodes[open_list[0]].edges[i]->passable == true){
				//Let Q be a valid node directly connected to P.
				temp_Q = findEndNode(open_list.at(0), all_nodes[open_list.at(0)].edges.at(i)->edge_id);

				//Check for Q’s presence on the Open List or Closed List
				if (std::find(open_list.begin(), open_list.end(), temp_Q) != open_list.end()){
					temp_g = all_nodes[open_list.at(0)].g_value + all_nodes[open_list.at(0)].edges.at(i)->cost;
					//If true, compare g values.
					//If the new g-value of Q is lower than the old g-value, change the parent of Q to P(this new route to Q is more efficient).
					if (temp_g < all_nodes[temp_Q].g_value){
						//Calculate g and h values for Q.
						all_nodes[temp_Q].g_value = temp_g;
						//Calculate the f-value of Q.
						all_nodes[temp_Q].f_value = all_nodes[temp_Q].heuristic + all_nodes[temp_Q].g_value;
						//Change the parent of Q
						all_nodes[temp_Q].parent_id = open_list.at(0);

						totalPathCost = all_nodes[temp_Q].g_value;
					}
				}
				if (std::find(closed_list.begin(), closed_list.end(), temp_Q) != closed_list.end()){
					//Else, do nothing (this new route to Q is no more efficient).
				}
				//Else, add Q to the Open List, assigning P as its parent.
				else if ((std::find(open_list.begin(), open_list.end(), temp_Q) == open_list.end()) && (std::find(closed_list.begin(), closed_list.end(), temp_Q) == closed_list.end())){
					//Update all of the values for Q
					all_nodes[temp_Q].g_value = all_nodes[open_list.at(0)].g_value + all_nodes[open_list.at(0)].edges.at(i)->cost;
					//Update the f value of Q
					all_nodes[temp_Q].f_value = all_nodes[temp_Q].heuristic + all_nodes[temp_Q].g_value;
					//Change the parent of Q to P
					all_nodes[temp_Q].parent_id = open_list.at(0);
					open_list.push_back(temp_Q);
					totalPathCost = all_nodes[temp_Q].g_value;
				}
			}
		}
		//Add the considered node into the closed list
		closed_list.push_back(open_list.at(0));
		//Remove the node that is being considered from the open list 
		open_list.erase(open_list.begin());
		//Sorts the vector using the comparison operator defined in the node sctruct
		std::sort(open_list.begin(), open_list.end());
	}
	return "Cannot proceed! List is empty!";

}

//User Interface
void UI(){

	int a = 0;
	cout << "Enter [ 1 ] to find detailed information of a given Node" << endl;
	cout << "Enter [ 2 ] to change the traversal cost of a given Edge" << endl;
	cout << "Enter [ 3 ] to make a given edge Impassable" << endl;
	cout << "Enter [ 4 ] to find a path from A to B" << endl;
	cout << "Enter [ 5 ] to Exit" << endl;
	cin >> a;

	if (a == 1){
		int x = 0;
		cout << "Enter the ID of a node" << endl;
		cin >> x;
		printNodeEdges(x);
	}
	if (a == 2){
		int x = 0;
		cout << "Enter the ID of an edge" << endl;
		cin >> x;
		int y = 0;
		cout << "Enter the NEW cost of the edge" << endl;
		cin >> y;
		for (int i = 0; i < 60; i++){
			for (int j = 0; j < 3; j++){
				if (all_nodes[i].edges[j]->edge_id == x){
					all_nodes[i].edges[j]->cost = y;
				}
			}
		}
	}
	if (a == 3){
		int x = 0;
		cout << "Enter the ID of an edge" << endl;
		cin >> x;
		char y;
		cout << "Do you want to make this edge Imassapble?" << endl;
		cin >> y;
		for (int i = 0; i < 60; i++){
			for (int j = 0; j < 3; j++){
				if (all_nodes[i].edges[j]->edge_id == x){
					if (y == 'y'){
						all_nodes[i].edges[j]->passable = false;
					}
					else if (y == 'n'){
						all_nodes[i].edges[j]->passable = true;
					}
				}
			}
		}
	}
	if (a == 4){
		int anode;
		int bnode;
		cout << "Enter starting node A" << endl;
		cin >> anode;
		cout << "Enter destination node B" << endl;
		cin >> bnode;
		calculateH(bnode);
		cout << aStarAlgorithm(anode, bnode) << endl;
	}
	if (a != 5){
		UI();
	}

	
}
void main(void) {

	createNodePositions();

	createNodes();

	findSmallestN2NDistance();

	findEdges();

	UI();


	/*Window w = Window(800, 600);
	Renderer r(w);

	Mesh*	m	= Mesh::LoadMeshFile("cube.asciimesh");
	Shader* s	= new Shader("basicvert.glsl", "basicFrag.glsl");

	if (s->UsingDefaultShader()) {
	cout << "Warningnode_positions[] = Vector3Using default shader! Your shader probably hasn't worked..." << endl;
	cout << "Press any key to continue." << endl;
	std::cin.get();
	}



	RenderObject o(m,s);
	o.SetModelMatrix(Matrix4::Translation(Vector3(0,0,-10)) * Matrix4::Scale(Vector3(1,1,1)));
	r.AddRenderObject(o);

	r.SetProjectionMatrix(Matrix4::Perspective(1, 100, 1.33f, 45.0f));

	r.SetViewMatrix(Matrix4::BuildViewMatrix(Vector3(0, 0, 0), Vector3(0, 0, -10)));

	while(w.UpdateWindow()) {
	float msec = w.GetTimer()->GetTimedMS();

	o.SetModelMatrix(o.GetModelMatrix() * Matrix4::Rotation(0.1f * msec,Vector3(0,0,1)));

	r.UpdateScene(msec);
	r.ClearBuffers();
	r.RenderScene();
	r.SwapBuffers();
	}
	delete m;
	delete s;*/
}