/*
CS421 Assignment 2

This program solves the 8-Puzzle using the following search algorithms:
Bounded Depth-First Search,
Iterative Deepening Search,
Breadth-First Search,
Best-First Search (A*)

*/
#include <iostream> 
#include <list>  
#include <ctime> 

using namespace std;

//8-Puzzle is a 3X3 matrix
const int n = 3;
//used in state compare operator function
bool informed_search = false;
//used in Expand function
bool bfs_search = false;
/*
Goal Configuration
	-------------
	| 1 | 2 | 3 |
	-------------
	| 8 | 0 | 4 |
	-------------
	| 7 | 6 | 5 |
	-------------
*/
int Goal[n][n] = { { 1, 2, 3 },{ 8, 0, 4 },{ 7, 6, 5 } };
//tracks generated nodes in search process
int nodes = 1;
//tracks runtime of search
int runtime = 1, timer = 0;

class State {
public:
	int A[n][n], g, h, total_cost, heuristic_select;
	State *parent;

	State();
	bool is_goal();
	bool operator==(const State &) const;
	bool operator<(const State &) const;
	void print();
	void heuristics(); //General Heuristic function - calls one of the heuristic functions listed below  
	//Heuristic Functions
	int manhattanHeuristic();
	int tilesOutOfPlace();
	int H();
	int chebyshevHeuristic();
};

State::State() {
	g = h = total_cost = 0;
	parent = NULL;
}

bool State::is_goal() {

	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			// if any two items not equal - not goal       
			if (A[i][j] != Goal[i][j])
				return false;
		}
	}
	//reaching this point means all items are equal - goal state found  
	return true;
}

bool State::operator==(const State &r) const {

	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			//if any two items not equal - not goal
			if (A[i][j] != r.A[i][j])
				return false;
		}
	}
	// states are equal  
	return true;
}

bool State::operator<(const State &r) const {

	if (informed_search) { // for heuristic based algorithms     
		return total_cost < r.total_cost;
	}
	else {
		return g < r.g; // for normal search algorithms   
	}
}

void State::print() {

	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++)
			cout << A[i][j] << ' ';
		cout << endl;
	}
	cout << endl;
}

void State::heuristics() {

	switch (heuristic_select) {
	case 1:
		h = tilesOutOfPlace();
		break;
	case 2:
		h = manhattanHeuristic();
		break;
	case 3:
		h = H();
		break;
	case 4:
		h = chebyshevHeuristic();
		break;
	}
}

int State::manhattanHeuristic() {
	int h_val = 0;
	bool found;
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			found = false;
			for (int i2 = 0; i2 < n; i2++) {
				for (int j2 = 0; j2 < n; j2++) {

					if (Goal[i][j] == A[i2][j2]) {
						//dx + dy             
						h_val += abs(i - i2) + abs(j - j2);
						found = true;
					}
					if (found)
						break;
				}
				if (found)
					break;
			}
		}
	}
	return h_val;
}

int State::tilesOutOfPlace() {
	int h_val = 0;

	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {

			if (Goal[i][j] != A[i][j])
				h_val += 1;
		}
	}
	return h_val;
}

int State::H() {

	int totdist = manhattanHeuristic();
	int seq = 0;
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			//check if empty tile
			if (A[i][j] == 0) {
				break;
			}
			//check if center tile
			if (i == 1 && j == 1) {
				if (A[i][j] != 0) {
					seq += 1;
				}
				break;
			}

			int next_tile, current_tile;
			//set 8 to 0 because 1 should follow 8
			A[i][j] == 8 ? current_tile = 0 : current_tile = A[i][j];

			//first row
			if (i == 0 && j != 2) {
				next_tile = A[i][j + 1];
			}
			//last column
			if (i != 2 && j == 2) {
				next_tile = A[i + 1][j];
			}
			//bottom row
			if (i == 2 && j != 0) {
				next_tile = A[i][j - 1];
			}
			//first column
			if (i != 0 && j == 0) {
				next_tile = A[i - 1][0];
			}
			//check if tiles are in sequence
			if ((current_tile + 1) != next_tile) {
				seq += 2;
			}
		}
	}

	return totdist + 3 * seq;
}

int State::chebyshevHeuristic() {
	int h_val = 0;
	bool found;
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			found = false;
			for (int i2 = 0; i2 < n; i2++) {
				for (int j2 = 0; j2 < n; j2++) {

					if (Goal[i][j] == A[i2][j2]) {
						int max;
						abs(i - i2) > abs(j - j2) ? max = abs(i - i2) : max = abs(j - j2);
						h_val += max;
						found = true;
					}
					if (found)
						break;
				}
				if (found)
					break;
			}
		}
	}
	return h_val;
}

list< State > closed_list, active_list;
State start_state, current_state, temp_state;

void BoundedDFS(int depth);
void IterativeDepthFirstSearch();
void BreadthFirstSearch();
void Astar();
void Expand();
void PrintPath(State *s);
bool InClosed(State &s);

int main() {

	std::cout << "8-Puzzle Solver" << endl;
	std::cout << "---------------" << endl;
	cout << "Position Matrix" << endl;
	cout << "-------------" << endl;
	cout << "| 1 | 2 | 3 |" << endl;
	cout << "-------------" << endl;
	cout << "| 4 | 5 | 6 |" << endl;
	cout << "-------------" << endl;
	cout << "| 7 | 8 | 9 |" << endl;
	cout << "-------------" << endl;

	//User inputs board configuration
	int val1, val2, val3, val4, val5, val6, val7, val8, val9;

	std::cout << "Enter Puzzle Configuration: " << endl;
	std::cout << "Position 1: " << endl;
	std::cin >> start_state.A[0][0];
	std::cout << "Position 2: " << endl;
	std::cin >> start_state.A[0][1];
	std::cout << "Position 3: " << endl;
	std::cin >> start_state.A[0][2];
	std::cout << "Position 4: " << endl;
	std::cin >> start_state.A[1][0];
	std::cout << "Position 5: " << endl;
	std::cin >> start_state.A[1][1];
	std::cout << "Position 6: " << endl;
	std::cin >> start_state.A[1][2];
	std::cout << "Position 7: " << endl;
	std::cin >> start_state.A[2][0];
	std::cout << "Position 8: " << endl;
	std::cin >> start_state.A[2][1];
	std::cout << "Position 9: " << endl;
	std::cin >> start_state.A[2][2];


	std::cout << "---- Board Configuration ----" << endl;
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			cout << start_state.A[i][j] << " ";
		}
		cout << endl;
	}

	int search_select;
	cout << "Select Algorithm:" << endl;
	cout << "1. Bounded Depth-First Search" << endl;
	cout << "2. Iterative Deepening Search" << endl;
	cout << "3. Breadth-First Search" << endl;
	cout << "4. Best-First Search" << endl;
	cout << "> ";
	cin >> search_select;

	//initialize start state  
	start_state.g = 0; // start at root node  
	start_state.heuristics(); //set heuristic value   
	start_state.total_cost = start_state.g + start_state.h; //total cost  
	start_state.parent = NULL; // root node  

	//selecting the search algorithm 
	switch (search_select) {
	case 1:
		int depth;
		cout << "Enter depth: " << endl;
		cout << "> ";
		cin >> depth;
		timer = (clock() * 1000) / CLOCKS_PER_SEC;
		BoundedDFS(depth);
		break;
	case 2:
		timer = (clock() * 1000) / CLOCKS_PER_SEC;
		IterativeDepthFirstSearch();
		break;
	case 3:
		bfs_search = true;
		timer = (clock() * 1000) / CLOCKS_PER_SEC;
		BreadthFirstSearch();
		break;
	case 4:
		informed_search = true;
		int heuristic_select;
		cout << "Select Heuristic: " << endl;
		cout << "1. number of tiles out of place" << endl;
		cout << "2. Manhattan distance" << endl;
		cout << "3. H" << endl;
		cout << "4. Chebyshev distance" << endl;
		cout << "> ";
		cin >> heuristic_select;
		if (heuristic_select > 0 && heuristic_select < 5) {
			start_state.heuristic_select = heuristic_select;
			timer = (clock() * 1000) / CLOCKS_PER_SEC;
			Astar();
		}
	}
	return 0;
}

void BoundedDFS(int depth) {

	std::cout << "Starting Bounded DFS Algorithm... \n";

	current_state = start_state;
	active_list.push_front(current_state);
	while (!active_list.empty())
	{
		current_state = active_list.front();

		if (current_state.is_goal()) {

			runtime = ((clock() * 1000) / CLOCKS_PER_SEC) - timer;
			// print search costs         
			cout << "Time = " << runtime << "ms\n";
			cout << "Nodes = " << nodes << "\n";
			cout << "Moves = " << current_state.g << "\n";
			//print the solution path         
			cout << "Path:\n";
			PrintPath(&current_state);
			return;
		} //if state not the goal and in the search depth.       
		else if (depth > current_state.g) {
			Expand();
		}
		else {
			active_list.pop_front();
		}
	}

	//if here - solution not found in specified depth
	std::cout << "Solution Not Found";

}

void IterativeDepthFirstSearch() {

	int depth = 0;
	cout << "Starting Iterative Depth First Search Algorithm... \n";
	while (true) {
		current_state = start_state;
		active_list.push_front(current_state);
		while (!active_list.empty())
		{

			current_state = active_list.front();

			if (current_state.is_goal()) {

				runtime = ((clock() * 1000) / CLOCKS_PER_SEC) - timer;
				//print the search costs        
				cout << "Time = " << runtime << "ms\n";
				cout << "Nodes = " << nodes << "\n";
				cout << "Moves = " << current_state.g << "\n";
				//print the solution path         
				cout << "Path:\n";
				PrintPath(&current_state);
				return;
			} //continue searching child nodes if depth not reached      
			else if (depth > current_state.g) {
				Expand();
			}
			else {
				active_list.pop_front();
			}
		}
		// clear both lists for the next round    
		active_list.clear();
		closed_list.clear();
		// increase the search depth     
		depth++;
	}
}

void BreadthFirstSearch() {
	cout << "Starting BFS Algorithm... \n";

	current_state = start_state;
	active_list.push_front(current_state);
	while (!active_list.empty())
	{
		current_state = active_list.front();

		if (current_state.is_goal()) {

			runtime = ((clock() * 1000) / CLOCKS_PER_SEC) - timer;
			// print search costs         
			cout << "Time = " << runtime << "ms\n";
			cout << "Nodes = " << nodes << "\n";
			cout << "Moves = " << current_state.g << "\n";
			//print the solution path         
			cout << "Path:\n";
			PrintPath(&current_state);
			return;
		}
		else { //continue searching child nodes
			Expand();
		}
	}

	//if here - solution not found
	std::cout << "Solution Not Found";
}


void Astar() {

	cout << "starting A* Algorithm... \n";
	current_state = start_state;
	active_list.push_front(current_state);
	while (true) {

		current_state = active_list.front();
		for (list<State>::iterator it = active_list.begin(); it != active_list.end(); ++it) {
			// find state with minimum total cost       
			if ((*it) < current_state) {
				current_state = (*it);
			}
		}

		if (current_state.is_goal()) {
			// calculate and print the search costs       
			runtime = ((clock() * 1000) / CLOCKS_PER_SEC) - timer;
			cout << "Time = " << runtime << "ms\n";
			cout << "Nodes = " << nodes << "\n";
			cout << "Moves = " << current_state.g << "\n";
			// print the solution path      
			cout << "Path:\n";
			PrintPath(&current_state);
			return;
		}
		else { //continue searching child nodes      
			Expand();
		}
	}
}

void Expand() {

	closed_list.push_back(current_state);

	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			//find blank tile in state array       
			if (current_state.A[i][j] == 0) {
				// if not in the first row         
				if (i > 0) {
					//set child          
					temp_state = current_state;
					temp_state.parent = &(closed_list.back());
					// shift blank tile UP           
					swap(temp_state.A[i][j], temp_state.A[i - 1][j]);
					// search for the child in the closed list           
					// if the child not found in the closed list           
					if (!InClosed(temp_state)) {
						temp_state.g += 1;
						temp_state.heuristics();
						temp_state.total_cost = temp_state.g + temp_state.h;
						//BFS requires FIFO
						if (bfs_search) {
							active_list.push_back(temp_state);
						}
						else {
							active_list.push_front(temp_state);
						}
						nodes++;
					}
				}
				//if not in last row   
				if (i < n - 1) {
					temp_state = current_state;
					temp_state.parent = &(closed_list.back());
					//shift the blank tile DOWN           
					swap(temp_state.A[i][j], temp_state.A[i + 1][j]);
					if (!InClosed(temp_state)) {
						temp_state.g += 1;
						temp_state.heuristics();
						temp_state.total_cost = temp_state.g + temp_state.h;
						//BFS requires FIFO
						if (bfs_search) {
							active_list.push_back(temp_state);
						}
						else {
							active_list.push_front(temp_state);
						}
						nodes++;
					}
				}
				// if not in the first column         
				if (j > 0) {
					temp_state = current_state;
					temp_state.parent = &(closed_list.back());
					// shift blank tile LEFT           
					swap(temp_state.A[i][j], temp_state.A[i][j - 1]);
					if (!InClosed(temp_state)) {
						temp_state.g += 1;
						temp_state.heuristics();
						temp_state.total_cost = temp_state.g + temp_state.h;
						//BFS requires FIFO
						if (bfs_search) {
							active_list.push_back(temp_state);
						}
						else {
							active_list.push_front(temp_state);
						}
						nodes++;
					}
				}// if not in the last column         
				if (j < n - 1) {
					temp_state = current_state;
					temp_state.parent = &(closed_list.back());
					// shift blank tile RIGHT          
					swap(temp_state.A[i][j], temp_state.A[i][j + 1]);
					if (!InClosed(temp_state)) {
						temp_state.g += 1;
						temp_state.heuristics();
						temp_state.total_cost = temp_state.g + temp_state.h;
						//BFS require FIFO
						if (bfs_search) {
							active_list.push_back(temp_state);
						}
						else {
							active_list.push_front(temp_state);
						}
						nodes++;
					}
				}
			}
		}
	}

	active_list.remove(current_state);
}

void PrintPath(State *s) {

	if (s != NULL) {
		// print current state   
		(*s).print();
		//recursively call printing its parent     
		PrintPath((*s).parent);
	}
}

bool InClosed(State &s) {
	for (list<State>::iterator it = closed_list.begin(); it != closed_list.end(); ++it) {
		if ((*it) == s) {
			return true;
		}
	}
	return false;
}
