#include "Maze_Traversal.h"
#define rows 16
#define cols 16
#define ARBITRARY_LARGE_NUMBER 999

/*
Notes:
This file includes functions utilized for maze traversal.
The maze may have more than one solution to it and is designed such that 
wall-following algorithms will not solve it. 
The maze traversal algorithm is A*.
The heuristic function for A* is the euclidean distance.
Micromouse mazes are 16 x 16 squares where the destination is any of the center 4
and the starting point is any of the 4 corners. There is only one gateway into
the center square. The starting point is bordered on all but one side by walls.
Since the starting point only has one exit, the Micromouse can determine the 
2 possible corners it can be in. Pick one. Mirror the maze if necessary wrong corner.
Verification of starting corner happens when Micromouse finds first opportunity to turn.
The maze will be stored in a 2D array of chars will values of:
* empty
* wall
* goal
* marked
* start
The outer rows and columns will be walls as the mouse cannot traverse through them
Odd rows and columns can only be empty, wall, or marked. 
Even rows and columns can only be empty, goal, or marked.
For 16x16 squares, a 33 x 33 char array can be used (see sample_maze.txt).
*/


/* GLOBAL VARIABLES FOR MAZE TRAVERSAL AND MOUSE ORIENTATION */
// simulator starts in bottom left corner
terrain maze[rows * 2 + 1][cols * 2 + 1];   // stores maze
MazeCell a_star_scores[rows][cols];         // stores visited cells
int start_maze_row = rows * 2 - 1;          // row of start cell in maze
int start_maze_col = 1;                     // col of start cell in maze
int maze_curr_row = start_maze_row;         // current row in maze
int maze_curr_col = start_maze_col;         // current col in maze
int start_a_star_scores_row = rows - 1;     // row of start cell in a_star_scores
int start_a_star_scores_col = 0;            // col of start cell in a_star_scores
int a_star_scores_curr_row = rows - 1;      // current row in a_star_scores
int a_star_scores_curr_col = 0;             // current col in a_star_scores
MazeCell a_star_start_cell(start_a_star_scores_row, start_a_star_scores_col);
MazeCell a_star_end_cell;                 // to be filled in after tremaux_explore is called
int a_star_end_row;
int a_star_end_col;
direction dir = up;                         // direction mouse is facing
std::stack<MazeCell> junctions;             // initialize a stack of MazeCells to store junctions
std::stack<MazeCell> pastCells;             // initialize a stack to store past moves 
std::stack<direction> pastDirections;       // initialize a stack to store past moves' directions
std::vector<MoveInstruction> a_star_path;   // vector of path to start determined by a_star_traverse
std::vector<MoveInstruction> fwd_path;      // vector of path to goal determined by a_star_traverse

/* TERRAIN FUNCTIONS */
// returns terrain of cell in front of the mouse
// direction is relative to mouse orientation
// if dir is not valid then it will return the terrain of the current cell
terrain get_front_cell_terrain() {
    switch(dir) {
        case up:
            return maze[maze_curr_row - 1][maze_curr_col];
        case down:
            return maze[maze_curr_row + 1][maze_curr_col];
        case left:
            return maze[maze_curr_row][maze_curr_col - 1];
        case right:
            return maze[maze_curr_row][maze_curr_col + 1];
        default:
            return maze[maze_curr_row][maze_curr_col];
    }
}

// returns terrain of cell behind of the mouse
// direction is relative to mouse orientation
// if dir is not valid then it will return the terrain of the current cell
terrain get_behind_cell_terrain() {
    switch(dir) {
        case up:
            return maze[maze_curr_row + 1][maze_curr_col];
        case down:
            return maze[maze_curr_row - 1][maze_curr_col];
        case left:
            return maze[maze_curr_row][maze_curr_col + 1];
        case right:
            return maze[maze_curr_row][maze_curr_col - 1];
        default:
            return maze[maze_curr_row][maze_curr_col];
    }
}

// returns terrain of cell to the left of the mouse
// direction is relative to mouse orientation
// if dir is not valid then it will return the terrain of the current cell
terrain get_left_cell_terrain() {
    switch(dir) {
        case up:
            return maze[maze_curr_row][maze_curr_col - 1];
        case down:
            return maze[maze_curr_row][maze_curr_col + 1];
        case left:
            return maze[maze_curr_row + 1][maze_curr_col];
        case right:
            return maze[maze_curr_row - 1][maze_curr_col];
        default:
            return maze[maze_curr_row][maze_curr_col];
    }
}

// returns terrain of cell to the right of the mouse
// direction is relative to mouse orientation
// if dir is not valid then it will return the terrain of the current cell
terrain get_right_cell_terrain() {
    switch(dir) {
        case up:
            return maze[maze_curr_row][maze_curr_col + 1];
        case down:
            return maze[maze_curr_row][maze_curr_col - 1];
        case left:
            return maze[maze_curr_row - 1][maze_curr_col];
        case right:
            return maze[maze_curr_row + 1][maze_curr_col];
        default:
            return maze[maze_curr_row][maze_curr_col];
    }
}

// returns terrain of cell in front of the mouse
// direction is relative to mouse orientation
// if dir is not valid then it will return the terrain of the current cell
terrain get_front_MazeCell_terrain() {
    switch(dir) {
        case up:
            return maze[maze_curr_row - 2][maze_curr_col];
        case down:
            return maze[maze_curr_row + 2][maze_curr_col];
        case left:
            return maze[maze_curr_row][maze_curr_col - 2];
        case right:
            return maze[maze_curr_row][maze_curr_col + 2];
        default:
            return maze[maze_curr_row][maze_curr_col];
    }
}

// returns terrain of cell behind of the mouse
// direction is relative to mouse orientation
// if dir is not valid then it will return the terrain of the current cell
terrain get_behind_MazeCell_terrain() {
    switch(dir) {
        case up:
            return maze[maze_curr_row + 2][maze_curr_col];
        case down:
            return maze[maze_curr_row - 2][maze_curr_col];
        case left:
            return maze[maze_curr_row][maze_curr_col + 2];
        case right:
            return maze[maze_curr_row][maze_curr_col - 2];
        default:
            return maze[maze_curr_row][maze_curr_col];
    }
}

// returns terrain of cell to the left of the mouse
// direction is relative to mouse orientation
// if dir is not valid then it will return the terrain of the current cell
terrain get_left_MazeCell_terrain() {
    switch(dir) {
        case up:
            return maze[maze_curr_row][maze_curr_col - 2];
        case down:
            return maze[maze_curr_row][maze_curr_col + 2];
        case left:
            return maze[maze_curr_row + 2][maze_curr_col];
        case right:
            return maze[maze_curr_row - 2][maze_curr_col];
        default:
            return maze[maze_curr_row][maze_curr_col];
    }
}

// returns terrain of cell to the right of the mouse
// direction is relative to mouse orientation
// if dir is not valid then it will return the terrain of the current cell
terrain get_right_MazeCell_terrain() {
    switch(dir) {
        case up:
            return maze[maze_curr_row][maze_curr_col + 2];
        case down:
            return maze[maze_curr_row][maze_curr_col - 2];
        case left:
            return maze[maze_curr_row - 2][maze_curr_col];
        case right:
            return maze[maze_curr_row + 2][maze_curr_col];
        default:
            return maze[maze_curr_row][maze_curr_col];
    }
}


// returns terrain of mouse's current cell
terrain get_current_cell() {
    return maze[maze_curr_row][maze_curr_col];
}

// returns terrain of cell one up from the mouse 
// direction is not relative to mouse orientation
// if parameters are provided, then it return
terrain get_absolute_up_cell_terrain() {
    return maze[maze_curr_row - 1][maze_curr_col];
}

// returns terrain of cell one down from mouse 
// direction is not relative to mouse orientation
// if parameters are provided, then it return
terrain get_absolute_down_cell_terrain() {
    return maze[maze_curr_row + 1][maze_curr_col];
}

// returns terrain of cell one left from mouse 
// direction is not relative to mouse orientation
// if parameters are provided, then it return
terrain get_absolute_left_cell_terrain() {
    return maze[maze_curr_row][maze_curr_col - 1];
}

// returns terrain of cell one right from mouse 
// direction is not relative to mouse orientation
// if parameters are provided, then it return
terrain get_absolute_right_cell_terrain() {
    return maze[maze_curr_row][maze_curr_col + 1];
}

// returns the terrain of the cell above
// the cell with the specified row and column
terrain get_up_cell_terrain(int r, int c) {
    return maze[r - 1][c];
}

// returns the terrain of the cell below
// the cell with the specified row and column
terrain get_down_cell_terrain(int r, int c) {
    return maze[r + 1][c];
}

// returns the terrain of the cell left
// the cell with the specified row and column
terrain get_left_cell_terrain(int r, int c) {
    return maze[r][c - 1];
}

// returns the terrain of the cell right
// the cell with the specified row and column
terrain get_right_cell_terrain(int r, int c) {
    return maze[r][c + 1];
}

// returns the MazeCell directly above the specified MazeCell
// @param mc reference MazeCell
MazeCell get_absolute_up_MazeCell(MazeCell mc) {
    return a_star_scores[mc.row - 1][mc.col];
}

// returns the MazeCell directly below the specified MazeCell
// @param mc reference MazeCell
MazeCell get_absolute_down_MazeCell(MazeCell mc) {
    return a_star_scores[mc.row + 1][mc.col];
}

// returns the MazeCell directly left of the specified MazeCell
// @param mc reference MazeCell
MazeCell get_absolute_left_MazeCell(MazeCell mc) {
    return a_star_scores[mc.row][mc.col - 1];
}

// returns the MazeCell directly right of the specified MazeCell
// @param mc reference MazeCell
MazeCell get_absolute_right_MazeCell(MazeCell mc) {
    return a_star_scores[mc.row][mc.col + 1];
}

// referencing the mouse's current coordinates, sets cell above (up), below (down), 
// left (left), right (right), or the current cell (no_direction) to the specified terrain 
// it will only set if terrain currently there is not already goal or start
// direction is absolute and  irrelevant to mouse's oriention
// direction refers to the location of the target cell relative to the mouse's current location
// returns true upon successful set
// returns false upon unsuccessful set
bool set_adjacent_or_current_cell(terrain t, direction d) {
    switch(d) {
        case up:
            if (maze[maze_curr_row - 1][maze_curr_col] != goal && maze[maze_curr_row - 1][maze_curr_col] != start) {
                maze[maze_curr_row - 1][maze_curr_col] = t;
                return true;
            }
            return false;
        case down:
            if (maze[maze_curr_row + 1][maze_curr_col] != goal && maze[maze_curr_row + 1][maze_curr_col] != start) {
                maze[maze_curr_row + 1][maze_curr_col] = t;
                return true;
            }
            return false;
        case left:
            if (maze[maze_curr_row][maze_curr_col - 1] != goal && maze[maze_curr_row][maze_curr_col - 1] != start) {
                maze[maze_curr_row][maze_curr_col - 1] = t;
                return true;
            }
            return false;
        case right:
            if (maze[maze_curr_row][maze_curr_col + 1] != goal && maze[maze_curr_row][maze_curr_col + 1] != start) {
                maze[maze_curr_row][maze_curr_col + 1] = t;
                return true;
            }
            return false;
        case no_direction:
            if (maze[maze_curr_row][maze_curr_col] != goal && get_current_cell() != start) {
                maze[maze_curr_row][maze_curr_col] = t;
                return true;
            }
            return false;
        default:
            return false;
    }
}

// sets cell to left of mouse to terrain t
// it will only set if terrain currently there is not already goal or start
// direction is in reference to the mouse's orientation. 
bool set_left_cell(terrain t) {
    switch(dir) {
         case up:
            if (maze[maze_curr_row][maze_curr_col - 1] != goal && maze[maze_curr_row][maze_curr_col - 1] != start) {
                maze[maze_curr_row][maze_curr_col - 1] = t;
                return true;
            }
            return false;
        case down:
            if (maze[maze_curr_row][maze_curr_col + 1] != goal && maze[maze_curr_row][maze_curr_col + 1] != start) {
                maze[maze_curr_row][maze_curr_col + 1] = t;
                return true;
            }
            return false;
        case left:
            if (maze[maze_curr_row + 1][maze_curr_col] != goal && maze[maze_curr_row + 1][maze_curr_col] != start) {
                maze[maze_curr_row + 1][maze_curr_col] = t;
                return true;
            }
            return false;
        case right:
            if (maze[maze_curr_row - 1][maze_curr_col] != goal && maze[maze_curr_row - 1][maze_curr_col] != start) {
                maze[maze_curr_row - 1][maze_curr_col] = t;
                return true;
            }
            return false;
        case no_direction:
            if (maze[maze_curr_row][maze_curr_col] != goal && get_current_cell() != start) {
                maze[maze_curr_row][maze_curr_col] = t;
                return true;
            }
            return false;
        default:
            return false;
    }
}

// sets cell to right of mouse to terrain t
// direction is in reference to the mouse's orientation. 
bool set_right_cell(terrain t) {
switch(dir) {
         case up:
            if (maze[maze_curr_row][maze_curr_col + 1] != goal && maze[maze_curr_row][maze_curr_col + 1] != start) {
                maze[maze_curr_row][maze_curr_col + 1] = t;
                return true;
            }
            return false;
        case down:
            if (maze[maze_curr_row][maze_curr_col - 1] != goal && maze[maze_curr_row][maze_curr_col - 1] != start) {
                maze[maze_curr_row][maze_curr_col - 1] = t;
                return true;
            }
            return false;
        case left:
            if (maze[maze_curr_row - 1][maze_curr_col] != goal && maze[maze_curr_row - 1][maze_curr_col] != start) {
                maze[maze_curr_row - 1][maze_curr_col] = t;
                return true;
            }
            return false;
        case right:
            if (maze[maze_curr_row + 1][maze_curr_col] != goal && maze[maze_curr_row + 1][maze_curr_col] != start) {
                maze[maze_curr_row + 1][maze_curr_col] = t;
                return true;
            }
            return false;
        case no_direction:
            if (maze[maze_curr_row][maze_curr_col] != goal && get_current_cell() != start) {
                maze[maze_curr_row][maze_curr_col] = t;
                return true;
            }
            return false;
        default:
            return false;
    }
}

// sets cell in front of mouse to terrain t
// direction is in reference to the mouse's orientation. 
bool set_front_cell(terrain t) {
switch(dir) {
         case up:
            if (maze[maze_curr_row - 1][maze_curr_col] != goal && maze[maze_curr_row - 1][maze_curr_col] != start) {
                maze[maze_curr_row - 1][maze_curr_col] = t;
                return true;
            }
            return false;
        case down:
            if (maze[maze_curr_row + 1][maze_curr_col] != goal && maze[maze_curr_row + 1][maze_curr_col] != start) {
                maze[maze_curr_row + 1][maze_curr_col] = t;
                return true;
            }
            return false;
        case left:
            if (maze[maze_curr_row][maze_curr_col - 1] != goal && maze[maze_curr_row][maze_curr_col - 1] != start) {
                maze[maze_curr_row][maze_curr_col - 1] = t;
                return true;
            }
            return false;
        case right:
            if (maze[maze_curr_row][maze_curr_col + 1] != goal && maze[maze_curr_row][maze_curr_col + 1] != start) {
                maze[maze_curr_row][maze_curr_col + 1] = t;
                return true;
            }
            return false;
        case no_direction:
            if (maze[maze_curr_row][maze_curr_col] != goal && get_current_cell() != start) {
                maze[maze_curr_row][maze_curr_col] = t;
                return true;
            }
            return false;
        default:
            return false;
    }
}

/* INITIALIZATION FUNCTIONS */
void init_maze() {
    // left and right bordering walls
    for (int r = 0; r < rows * 2 + 1; r++) {
        maze[r][0] = wall;
        maze[r][cols * 2] = wall;
    }
    //std::cout << "left and right works\n";

    // top and bottom bordering walls
    for (int c = 1; c < cols * 2; c++) {
        maze[0][c] = wall;
        maze[rows * 2][c] = wall;
    }
    //std::cout << "top and bottom works\n";

    // middle goal squares
    maze[rows - 1][cols - 1] = goal;
    maze[rows - 1][cols + 1] = goal;
    maze[rows + 1][cols - 1] = goal;
    maze[rows + 1][cols + 1] = goal;  
    
    // squares between goals
    maze[rows - 1][cols] = empty;
    maze[rows + 1][cols] = empty;
    maze[rows][cols] = empty;
    maze[rows][cols - 1] = empty;
    maze[rows][cols + 1] = empty;

    //std::cout << "goals works\n";

    // populate walls in maze (even rows, even cols)
    for (int r = 2; r < rows * 2 + 1; r+=2) {
        for (int c = 2; c < cols * 2 + 1; c+=2) {
            maze[r][c] = wall;
        }
    }

    // initialize MazeCells
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            a_star_scores[r][c].g_score = ARBITRARY_LARGE_NUMBER;
            a_star_scores[r][c].f_score = ARBITRARY_LARGE_NUMBER;
            a_star_scores[r][c].row = r;
            a_star_scores[r][c].col = c;
        }
    }
    // choose starting square to be the bottom left corner (assuming mouse is already facing exit)
    maze[start_maze_row][start_maze_col] = start;
    a_star_scores[rows - 1][0].g_score = 0;
    a_star_scores[rows - 1][0].f_score = 0;

}

// initialize the mouse to make it face front. 
// direction will always begin as up
void init_mouse(Drivetrain* dr) {
    // make the mouse face front
    init_orientation_up(dr);

    bool verifyCorner = false;
    while (!verifyCorner && /*get_front_cell_terrain() != wall*/ !API::wallFront(dr)) {
        // start by moving forward as much as possible. Guaranteed to be able to move forward at least one.
        // stops moving forward when it is possible to make a turn. At this point, the mouse will update 
        // its starting corner as necessary.
        moveForward(dr, 1, true, false); 
        // log((char*)"moved forward 1 in API\n");
        if (/*get_left_cell_terrain() != wall &&*/ !API::wallRight(dr)) {
            // correct corner
            verifyCorner = true;
        } else if (/*get_right_cell_terrain() != wall ||*/ !API::wallLeft(dr)) {
            // swap corner by moving col 1 to right-most non-wall column
            for (int r = maze_curr_row; r < rows * 2; r++) {
                maze[r][cols * 2 - 1] = maze[r][1];
                maze[r][1] = empty;
            }
            // also update terrain from col 0 to cols * 2 - 2
            for (int r = maze_curr_row + 2; r < rows * 2; r+=2) {
                maze[r][cols * 2 - 2] = maze[r][0];
                maze[r][0] = wall;
            }
            // also update terrain from col 2 to original
            for (int r = maze_curr_row; r < rows * 2; r+=2) {
                maze[r][2] = empty;
            }
            a_star_scores[start_a_star_scores_row][start_a_star_scores_col].g_score = ARBITRARY_LARGE_NUMBER;
            a_star_scores[start_a_star_scores_row][start_a_star_scores_col].f_score = ARBITRARY_LARGE_NUMBER;
            // change start coord
            start_maze_col = cols * 2 - 1;
            start_a_star_scores_col = cols - 1;
            a_star_start_cell.col = start_a_star_scores_col;
            // change mouse position
            maze_curr_col = cols * 2 - 1;
            a_star_scores_curr_col = cols - 1;
            verifyCorner = true;
            a_star_scores[start_a_star_scores_row][start_a_star_scores_col].g_score = 0;
            a_star_scores[start_a_star_scores_row][start_a_star_scores_col].f_score = 0;
        }
        // add start to junction stack and pastCells stack
        junctions.push(MazeCell(start_a_star_scores_row, start_a_star_scores_col));
        //pastCells.push(MazeCell(start_a_star_scores_row, start_a_star_scores_col));
        for (int i = start_a_star_scores_row + 1; i < a_star_scores_curr_row; i++) {
            // add traversed cells to stack
            pastCells.push(MazeCell(i, start_a_star_scores_col));
            // add directions to stack
            pastDirections.push(up);
        }
    }
    // check if current cell is junction and if so, add it
    if ((get_front_cell_terrain() != wall && get_left_cell_terrain() != wall)
        || (get_front_cell_terrain() != wall && get_right_cell_terrain() != wall)
        || (get_left_cell_terrain() != wall && get_right_cell_terrain() != wall)) 
    {
        //log((char*) "set a junction!\n");
        set_adjacent_or_current_cell(junction, no_direction);
        // push the junction 
        junctions.push(MazeCell(a_star_scores_curr_row, a_star_scores_curr_col));
    } else {
        set_adjacent_or_current_cell(marked, no_direction);
    }
    // while (!pastCells.empty()) {
    //     MazeCell temp = pastCells.top();
    //     log((char*) "past Cells\n");
    //     printMazeCell(temp);
    //     pastCells.pop();
    // }
}

// makes the mouse face up in the maze
void init_orientation_up(Drivetrain* dr) {
    if (/*get_front_cell_terrain() == wall ||*/ !API::wallFront(dr)) {
        if (/*get_left_cell_terrain() != wall &&*/ !API::wallLeft(dr)) {
            turnLeft(dr);
        } else if (/*get_right_cell_terrain() != wall &&*/ !API::wallRight(dr)) {
            turnRight(dr);
        } else {
            turnLeft(dr);
            turnLeft(dr);
        }
    }
    // since it is always started in the bottom left corner, left and bottom
    // will already be wall. only need to set right.
    set_adjacent_or_current_cell(wall, right);
    dir = up;
}

/* CALCULATION FUNCTIONS */
// returns euclidean distance between two cartesian points (x0, y0) and (x1, y1)
double euclidean_distance(double x0, double y0, double x1, double y1)
{
    return sqrt(pow(x0 - x1, 2) + pow(y0 - y1, 2));
}

// returns manhattan distance between two cartesian points
double manhattan_distance(double x0, double y0, double x1, double y1) {
    return abs(x1 - x0) + abs(y1 - y0);
}

// returns the minimum double given two doubles
double min(double d0, double d1) {
    if (d0 > d1) {
        return d1;
    } 
    return d0;
}

// converts a_star_scores coordinate to maze coordinate
double conv_a_star_coord_to_maze_coord(int x) {
    return x * 2 + 1;
}

// converts maze coordinate to a_star coordinate
double conv_maze_coord_to_a_star_coord(int x) {
    return x / 2;
}

// returns corresponding MazeCell given maze coordinates
MazeCell conv_maze_coord_to_a_star_coord(int r, int c) {
    return MazeCell(conv_maze_coord_to_a_star_coord(r), conv_maze_coord_to_a_star_coord(c));
}

/* MOVEMENT FUNCTIONS */ 

// moves mouse forward
// @return true if successful
// @return false if not successful
bool moveForward(Drivetrain* dr, int distance, bool updateTerrain, bool isUndo) {
    int row_inc = 0;
    int col_inc = 0;
    // in maze, move forward 2 cells because one will be a wall row/column
    if (/*get_front_cell_terrain() == wall ||*/ API::wallFront(dr)) {
        log((char*)"failed moving: wall in front\n");
        return false;
    }
    // decide increment based on direction
    switch(dir) {
        case up:
            if (maze_curr_row - 2 * distance >= 0) {
                row_inc = -1;
            } else {
                log((char*)"failed moving up\n");
                return false;
            }
            break;
        case down:
            if (maze_curr_row + 2 * distance < rows * 2 + 2) {
                row_inc = 1;
            } else {
                log((char*)"failed moving down\n");
                return false;
            }
            break;
        case left:
            if (maze_curr_col - 2 * distance >= 0) {
                col_inc = -1;
            } else {
                log((char*)"failed moving left\n");
                return false;
            }
            break;
        case right:
            if (maze_curr_col + 2 * distance < cols * 2 + 2) {
                col_inc = 1;
            } else {
                log((char*)"failed moving right\n");
                return false;
            }
            break;
        default:
            log((char*)"failed moving default\n");
            return false;
    }

    for (int d = 0; d < distance; d++) {
        if (API::wallFront(dr)) {
            return false;
        }
        if (!isUndo) {
            pastCells.push(MazeCell(a_star_scores_curr_row, a_star_scores_curr_col));
            pastDirections.push(dir);
        }
        API::moveForward(dr);
        //log((char*)"API moved forward\n");
        maze_curr_row += row_inc;
        maze_curr_col += col_inc;
        // set current cell
        if (updateTerrain) {
            // log((char*)"Preparing to update terrain...\n");
            // update terrain
            if (get_current_cell() == empty) {
                // junction check not required since these cells will be in between two walls
                set_adjacent_or_current_cell(marked, no_direction);
            } else if (get_current_cell() == marked) {
                // already travelled once and now returned --> mark as double_marked
                set_adjacent_or_current_cell(double_marked);
            } 
            // other traversable options are junction, goal, start, or double_marked 
            // these should not change though if traversed on again
        }
        maze_curr_row += row_inc;
        maze_curr_col += col_inc;
        a_star_scores_curr_row += row_inc;
        a_star_scores_curr_col += col_inc;
        if (updateTerrain) {
            // set adjacent cells
            if (API::wallRight(dr)) {
                set_right_cell(wall);
            } 
            /*else {
                set_right_cell(marked);
            }*/
            if (API::wallLeft(dr)) {
                set_left_cell(wall);
            } 
            /*else {
                set_left_cell(marked);
            }
            */
            if (API::wallFront(dr)) {
                set_front_cell(wall);
            } 
            /*else {
                set_front_cell(marked);
            }
            */
            // set current cell
            if (get_current_cell() == empty) {
                // junction check
                if (!isUndo) {
                    // only do the junction check if not undoing
                    if ((get_front_cell_terrain() != wall && get_left_cell_terrain() != wall)
                    || (get_front_cell_terrain() != wall && get_right_cell_terrain() != wall)
                    || (get_left_cell_terrain() != wall && get_right_cell_terrain() != wall)
                    || (get_front_cell_terrain() != wall && get_left_cell_terrain() != wall)
                    || (get_front_cell_terrain() != wall && get_right_cell_terrain() != wall)
                    || (get_left_cell_terrain() != wall && get_right_cell_terrain() != wall)) 
                    {
                        //log((char*) "set a junction!\n");
                        set_adjacent_or_current_cell(junction, no_direction);
                        // push the junction 
                        junctions.push(MazeCell(a_star_scores_curr_row, a_star_scores_curr_col));
                    } else {
                        set_adjacent_or_current_cell(marked, no_direction);
                    }
                } else {
                    set_adjacent_or_current_cell(marked, no_direction);
                }
            } else if (get_current_cell() == marked) {
                // already travelled once and now returned --> mark as double_marked
                set_adjacent_or_current_cell(double_marked);
            } 
            // other traversable options are junction, goal, start, or double_marked 
            // these should not change though if traversed on again
        }
        //log((char*)"Updated terrain!\n");
    }
    // print_maze();
    // replace with Micromouse movement here
    return true;
}

// undo the last move. note that the direction will become the opposite of the original
// returns the direction mouse moved to undo
direction undoMove(Drivetrain* dr) {
    // get the last direction
    direction lastDir = pastDirections.top();
    pastDirections.pop();
    // get the last cell traveled to
    MazeCell lastCell = pastCells.top();
    pastCells.pop();
    moveToAdjacentCell(dr, lastCell, true, true);
    int row_inc = 0;
    int col_inc = 0;
    // decide increment based on direction
    switch(lastDir) {
        case up:
            row_inc = 1;
            lastDir = down;
            break;
        case down:
            row_inc = -1;
            lastDir = up;
            break;
        case left:
            col_inc = 1;
            lastDir = right;
            break;
        case right:
            col_inc = -1;
            lastDir = left;
            break;
        default:
            break;
    }
    //log((char*)"API undid move\n");
    // replace with Micromouse movement here
    return lastDir;
}

// micromouse turns right
bool turnRight(Drivetrain* dr) {
    //log((char*)"turning right\n");
    // TODO: replace with Micromouse movement here
    API::turnRight(dr);
    switch (dir) {
        case up:
            dir = right;
            return true;
        case down: 
            dir = left; 
            return true;
        case left:
            dir = up;
            return true;
        case right:
            dir = down;
            return true;
        default:
            return false;
    }
}

// micromouse turns left
bool turnLeft(Drivetrain* dr) {
    // log((char*)"turning left\n");
    // TODO: replace with Micromouse movement here
    API::turnLeft(dr);
    switch (dir) {
        case up:
            dir = left;
            return true;
        case down: 
            dir = right; 
            return true;
        case left:
            dir = down;
            return true;
        case right:
            dir = up;
            return true;
        default:
            return false;
    }
}

// makes mouse move up a distance and changes direction to up
bool moveUp(Drivetrain* dr, int distance, bool updateTerrain, bool isUndo) {
    switch (dir) {
        case up:
            break;
        case down: 
            turnLeft(dr);
            turnLeft(dr);
            break;
        case left:
            turnRight(dr);
            break;
        case right:
            turnLeft(dr);
            break;
        default:
            break;
    }
    dir = up;
    return moveForward(dr, distance, updateTerrain, isUndo);
}

// makes mouse move down a distance and changes direction to down
bool moveDown(Drivetrain* dr, int distance, bool updateTerrain, bool isUndo) {
    switch (dir) {
        case up:
            turnLeft(dr);
            turnLeft(dr);
            break;
        case down: 
            break;
        case left:
            turnLeft(dr);
            break;
        case right:
            turnRight(dr);
            break;
        default:
            break;
    }
    dir = down;
    return moveForward(dr, distance, updateTerrain, isUndo);
}

// makes mouse move left a distance and changes direction to left
bool moveLeft(Drivetrain* dr, int distance, bool updateTerrain, bool isUndo) {
    switch (dir) {
        case up:
            turnLeft(dr);
            break;
        case down: 
            turnRight(dr);
            break;
        case left:
            break;
        case right:
            turnLeft(dr);
            turnLeft(dr);
            break;
        default:
            break;
    }
    dir = left;
    return moveForward(dr, distance, updateTerrain, isUndo);
}

// makes mouse move right a distance and changes direction to right
bool moveRight(Drivetrain* dr, int distance, bool updateTerrain, bool isUndo) {
    switch (dir) {
        case up:
            turnRight(dr);
            break;
        case down: 
            turnLeft(dr);
            break;
        case left:
            turnLeft(dr);
            turnLeft(dr);
            break;
        case right:
            break;
        default:
            break;
    }
    return moveForward(dr, distance, updateTerrain, isUndo);
}

// mouse moves to a given target adjacent cell
// assumes that the target cell is adjacent already (no check)
bool moveToAdjacentCell(Drivetrain* dr, MazeCell target, bool updateTerrain, bool isUndo) {
    direction d;
    if (target.row == a_star_scores_curr_row) {
        if (target.col > a_star_scores_curr_col) {
            d = right;
        } else {
            d = left;
        }
    } else if (target.row > a_star_scores_curr_row) {
        d = down;
    } else {
        d = up;
    }
    //fprintf(stderr, "target: (%d, %d) current: (%d, %d) new direction: %d\n", target.row, target.col, a_star_scores_curr_row, a_star_scores_curr_col, d);
    //fflush(stderr);
    switch(d) {
        case up:
            return moveUp(dr, 1, updateTerrain, isUndo);
        case down:
            return moveDown(dr, 1, updateTerrain, isUndo);
        case left:
            return moveLeft(dr, 1, updateTerrain, isUndo);
        case right:
            return moveRight(dr, 1, updateTerrain, isUndo);
        default:
            return false;
    }
}

/* ALGORITHM */
// heuristic function for A*
// minimum euclidean distance between the current cell and 4 goal cells
double euclidean_distance_heuristic() {
    double goal0 = euclidean_distance(maze_curr_row, maze_curr_col, rows, cols);
    double goal1 = euclidean_distance(maze_curr_row, maze_curr_col, rows + 2, cols);
    double goal2 = euclidean_distance(maze_curr_row, maze_curr_col, rows, cols + 2);
    double goal3 = euclidean_distance(maze_curr_row, maze_curr_col, rows + 2, cols + 2);
    return min(min(goal0, goal1), min(goal2, goal3));
}

// minimum Manhattan distance between a MazeCell and 4 goal cells
double manhattan_distance_heuristic() {
    double goal0 = manhattan_distance(maze_curr_row, maze_curr_col, rows, cols);
    double goal1 = manhattan_distance(maze_curr_row, maze_curr_col, rows + 2, cols);
    double goal2 = manhattan_distance(maze_curr_row, maze_curr_col, rows, cols + 2);
    double goal3 = manhattan_distance(maze_curr_row, maze_curr_col, rows + 2, cols + 2);
    return min(min(goal0, goal1), min(goal2, goal3));
}

// minimum euclidean distance between a MazeCell and 4 goal cells
double euclidean_distance_heuristic(MazeCell mc0) {
    double goal0 = euclidean_distance(mc0.row, mc0.col, rows / 2, cols / 2);
    double goal1 = euclidean_distance(mc0.row, mc0.col, rows / 2 - 1, cols / 2);
    double goal2 = euclidean_distance(mc0.row, mc0.col, rows / 2 - 1, cols / 2 - 1);
    double goal3 = euclidean_distance(mc0.row, mc0.col, rows / 2, cols / 2 - 1);
    return min(min(goal0, goal1), min(goal2, goal3));
}

// minimum manhattan distance between a MazeCell and 4 goal cells
double manhattan_distance_heuristic(MazeCell mc0) {
    double goal0 = manhattan_distance(mc0.row, mc0.col, rows / 2, cols / 2);
    double goal1 = manhattan_distance(mc0.row, mc0.col, rows / 2 - 1, cols / 2);
    double goal2 = manhattan_distance(mc0.row, mc0.col, rows / 2 - 1, cols / 2 - 1);
    double goal3 = manhattan_distance(mc0.row, mc0.col, rows / 2, cols / 2 - 1);
    return min(min(goal0, goal1), min(goal2, goal3));
}

// minimum euclidean distance between two MazeCells
double euclidean_distance_heuristic(MazeCell mc0, MazeCell mc1) {
    return euclidean_distance(mc0.row, mc0.col, mc1.row, mc1.col);
}

// minimum manhattan distance between two MazeCells
double manhattan_distance_heuristic(MazeCell mc0, MazeCell mc1) {
    return manhattan_distance(mc0.row, mc0.col, mc1.row, mc1.col);
}

// uses the Tremaux algorithm to get a picture of the maze
void tremaux_explore(Drivetrain* dr) {
    // call init_mouse
    init_mouse(dr);
    int state = 0;
    MazeCell lastCell;
    MazeCell lastJunction;
    // while have not returned to original spot
    while (!(maze_curr_row == start_maze_row && maze_curr_col == start_maze_col))  {
        // while there is either a marked or empty adjacent cell, keep traversing
        // prioritize empty then marked, front, right, then left
        while (state != 3) {
            switch(state) {
                case 0: 
                    // log((char*)"state 0\n");
                    // checking for empty adjacent cells to traverse
                    if (get_front_cell_terrain() == empty && get_front_MazeCell_terrain() != junction) {
                        moveForward(dr, 1);
                    } else if (get_right_cell_terrain() == empty && get_right_MazeCell_terrain() != junction) {
                        turnRight(dr);
                        moveForward(dr, 1);
                    } else if (get_left_cell_terrain() == empty && get_left_MazeCell_terrain() != junction) {
                        turnLeft(dr);
                        moveForward(dr, 1);
                    }
                    else { // no adjacent empty cells
                        state = 2;
                    }
                    break;
                case 1: 
                // no adjacent empty cells to traverse
                // checking for marked adjacent cells to traverse
                    // log((char*)"state 1\n");
                    if (get_front_cell_terrain() == marked && get_front_MazeCell_terrain() != junction) {
                        moveForward(dr, 1);
                        state = 0; // go back and check for emptys again
                    } else if (get_right_cell_terrain() == marked && get_right_MazeCell_terrain() != junction) {
                        turnRight(dr);
                        moveForward(dr, 1);
                        state = 0; // go back and check for emptys again
                    } else if (get_left_cell_terrain() == marked && get_left_MazeCell_terrain() != junction) {
                        turnLeft(dr);
                        moveForward(dr, 1);
                        state = 0; // go back and check for emptys again
                    } else {
                        state = 1; // no adjacent marked cells
                    }
                    break;
                case 2: 
                // no adjacent empty or marked cells to traverse
                // backtrack
                    // log((char*)"state 2\n");
                    // pop off the last junction and undo until the previous junction
                    if (get_current_cell() == junction) {
                        junctions.pop();
                    } else {
                        // set current cell to double_marked
                        set_adjacent_or_current_cell(double_marked, no_direction);
                    }
                    // when there is nowhere possible to go, backtrack until the current cell is the last junction
                    lastJunction = junctions.top();
                    // printMazeCell(lastJunction);
                    while (!(a_star_scores_curr_row == lastJunction.row && a_star_scores_curr_col == lastJunction.col)) {
                        lastCell = pastCells.top();
                        // printMazeCell(lastCell);
                        undoMove(dr);
                    }
                    // check if the current cell is the start cell. 
                    if (maze_curr_row == start_maze_row && maze_curr_col == start_maze_col) {
                        // if so, then traversing is done. set state to 3
                        state = 3;
                    } else {
                        // otherwise keep traversing
                        state = 0;
                    }
                    break;
            }
        } 
        // log((char*)"state 3\n");     
    }
    a_star_end_cell = determine_goal_cell();
    a_star_end_row = a_star_end_cell.row;
    a_star_end_col = a_star_end_cell.col;
    // printMazeCell(a_star_end_cell);
}

std::vector<MoveInstruction> a_star_traverse() {
    // source: https://levelup.gitconnected.com/a-star-a-search-for-solving-a-maze-using-python-with-visualization-b0cae1c3ba92
    // priority queue info: https://www.geeksforgeeks.org/priority-queue-in-cpp-stl/# 
    // create a priority queue
    std::priority_queue<MazeCell*, std::vector<MazeCell*>, Compare_F_Score> pq;
    // f(n): total cost to reach cell n
    // g(n): actual cost to reach cell n from start cell
    // h(n): heuristic cost to reach goal cell from cell n
    // create variables for values of g(n), f(n), current cell, and child cell
    double g_score = ARBITRARY_LARGE_NUMBER; // assign large number to compare with
    double f_score = ARBITRARY_LARGE_NUMBER; // assign large number to compare with
    double temp_g_score, temp_f_score;      
    MazeCell* currCell = new MazeCell(start_a_star_scores_row, start_a_star_scores_col, 0, 0);
    MazeCell* childCell;
    std::priority_queue<MazeCell*, std::vector<MazeCell*>, Compare_F_Score> traversable_adjacent_cells;
    // place the total score, heuristic, and starting cell in priority queue
    pq.push(currCell);
    // printMazeCell(*currCell);
    MoveInstruction move;

    // while priority queue is not empty and the goal has been reached
    while (!pq.empty() && !(a_star_scores_curr_row == a_star_end_row && a_star_scores_curr_col == a_star_end_col)) {
        // get the current cell by getting from the priority queue
        currCell = pq.top();
        a_star_scores_curr_row = currCell->row;
        a_star_scores_curr_col = currCell->col;
        pq.pop();
        // log((char* ) "currCell: ");
        // printMazeCell(*currCell);
        // fprintf(stderr, "%0.1f %0.1f \n", currCell.g_score, temp_g_score);
        // get every possible traversable adjacent cell
        if (get_up_cell_terrain(conv_a_star_coord_to_maze_coord(currCell->row), conv_a_star_coord_to_maze_coord(currCell->col)) != wall) {
            // up
            childCell = &(a_star_scores[a_star_scores_curr_row - 1][a_star_scores_curr_col]);
            traversable_adjacent_cells.push(childCell);
        } 
        if (get_down_cell_terrain(conv_a_star_coord_to_maze_coord(currCell->row), conv_a_star_coord_to_maze_coord(currCell->col)) != wall) {
            // down
            childCell = &(a_star_scores[a_star_scores_curr_row + 1][a_star_scores_curr_col]);
            traversable_adjacent_cells.push(childCell);
        } 
        if (get_left_cell_terrain(conv_a_star_coord_to_maze_coord(currCell->row), conv_a_star_coord_to_maze_coord(currCell->col)) != wall) {
            // left
            childCell = &(a_star_scores[a_star_scores_curr_row][a_star_scores_curr_col - 1]);
            traversable_adjacent_cells.push(childCell);
        } 
        if (get_right_cell_terrain(conv_a_star_coord_to_maze_coord(currCell->row), conv_a_star_coord_to_maze_coord(currCell->col)) != wall) {
            // right
            childCell = &(a_star_scores[a_star_scores_curr_row][a_star_scores_curr_col + 1]);
            traversable_adjacent_cells.push(childCell);
        }
        // for every direction
            // childCell = next possible cell
            // temp g score = g_score currCell + 1
            // temp f score = temp_g_score + h(childCell)
            // if temp_f_score < f_score(childCell)
                // g_score(childCell) = temp_g_score
                // f_Score(childCell) = temp_f_score
                // place (f_Score(childCell), h(childCell), childCell) in priority queue
        while (!traversable_adjacent_cells.empty()) {
            childCell = traversable_adjacent_cells.top();
            traversable_adjacent_cells.pop();
            // log((char* ) "childCell: ");
            // printMazeCell(*childCell);
            temp_g_score = a_star_scores[currCell->row][currCell->col].g_score + 1;
            // fprintf(stderr, "h_score: %0.6f\n", euclidean_distance_heuristic(*childCell, a_star_end_cell));
            temp_f_score = temp_g_score + euclidean_distance_heuristic(*childCell, a_star_end_cell);
            // fprintf(stderr, "temp_g_score: %0.2f, temp_f_score: %0.1f, childCell.f_score: %0.1f\n", temp_g_score, temp_f_score, (*childCell).f_score);
            // fflush(stderr);
            // if score is less, update it
            if (temp_f_score < childCell->f_score) {
                childCell->g_score = temp_g_score;
                childCell->f_score = temp_f_score;
                pq.push(childCell);
                // path will be backwards from goal
                // store the childCell's next move as the current cell
                move.start = *childCell;
                move.end = *currCell;
                a_star_path.emplace_back(move);
            }
        }
    }
    // log((char*) "get path: ");
    // print_vector(a_star_path);
    // return fwd_path;
    // get the path, start from goal cell
    MazeCell curr = a_star_end_cell;
    // printMazeCell(curr);
    // while not at the start
    while (!(curr.row == a_star_start_cell.row && curr.col == a_star_start_cell.col)) {
        // find the MoveInstruction whose start matches curr
        for (int i = 0; i < a_star_path.size(); i++) {
            if (a_star_path.at(i).start.row == curr.row && a_star_path.at(i).start.col == curr.col) {
                move.start = a_star_path.at(i).end;
                move.end = a_star_path.at(i).start;
                fwd_path.emplace_back(move);
                curr = a_star_path.at(i).end;
                break;
            }
        }
    }
    a_star_scores_curr_row = a_star_start_cell.row;
    a_star_scores_curr_col = a_star_start_cell.col;
    return fwd_path;
}

void travelPath(Drivetrain* dr, std::vector<MoveInstruction> moves, bool forward) {
    if (forward) {
        for (int i = moves.size() - 1; i >= 0; i--) {
            moveToAdjacentCell(dr, moves.at(i).end, false, false);
        }
    } else {
        for (int i = 0; i < moves.size(); i++) {
            moveToAdjacentCell(dr, moves.at(i).start, false, false);
        }
    }
}

// identifies which of the 4 goal cells is adjacent to the opening (only has 1 wall)
// for this to work properly, it is assumed that tremaux_explore() has been run first and maze is fully explored
MazeCell determine_goal_cell() {
    // if top left corner has wall top and wall left, it's not the destination. 
    // otherwise if one isn't wall, it's the destination
    if (maze[rows - 2][cols - 1] != wall || maze[rows - 1][cols - 2] != wall) {
        return MazeCell(rows / 2 - 1, cols / 2 - 1);
    }
    // if top right corner has wall top and wall right, it's not the destination. 
    // otherwise if one isn't wall, it's the destination
    if (maze[rows - 2][cols + 1] != wall || maze[rows - 1][cols + 2] != wall) {
        return MazeCell(rows / 2 - 1, cols / 2);
    }
    // if bottom right corner has wall bottom and wall right, it's not the destination. 
    // otherwise if one isn't wall, it's the destination
    if (maze[rows + 2][cols + 1] != wall || maze[rows + 1][cols + 2] != wall) {
        return MazeCell(rows / 2, cols / 2);
    }
    // if bottom left corner has wall bottom and wall left, it's not the destination. 
    // otherwise if one isn't wall, it's the destination
    if (maze[rows + 2][cols - 1] != wall || maze[rows + 1][cols - 2] != wall) {
        return MazeCell(rows / 2, cols / 2 - 1);
    }
    // default return top left
    return MazeCell(0, 0);
}

/* MISCELLANEOUS FUNCTIONS */

// prints the maze to the terminal
void print_maze() {
    for (int r = 0; r < rows * 2 + 1; r++) {
        for (int c = 0; c < cols * 2 + 1; c++) {
            if (r != maze_curr_row || c != maze_curr_col) {
                fprintf(stderr, "%d ", maze[r][c]);
                fflush(stderr);
            } else {
                fprintf(stderr, "M ");
                fflush(stderr);
            }
            //std::cout << maze[r][c] << ' ';
        }
        fprintf(stderr,"\n");
        //std::cout << '\n';
    }
}

void print_a_star_scores() {
    log((char* ) "[F_score, G_score]; \n");
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            fprintf(stderr, "[%.2f, %.2f]; ", a_star_scores[r][c].f_score, a_star_scores[r][c].g_score);
            fflush(stderr);
        }
        log((char*) "\n");
    }
}

void printMazeCell(MazeCell mc) {
    fprintf(stderr, "(%d, %d): [%.2f, %.2f]; \n", 
    mc.row, 
    mc.col, 
    a_star_scores[mc.row][mc.col].f_score, 
    a_star_scores[mc.row][mc.col].g_score);
    fflush(stderr);
}

void log(char* text) {
    fprintf(stderr, "%s\n", text);
    fflush(stderr);
}

void log_maze() {
    char* temp = (char*) malloc(sizeof(char) * 3);
    *(temp + 1) = ' ';
    *(temp + 2) = '\0';
    char* newline = (char*) malloc(sizeof(char));
    *newline = '\n';
    for (int r = 0; r < rows * 2 + 1; r++) {
        for (int c = 0; c < cols * 2 + 1; c++) {
            *temp = maze[r][c];
            log(temp);
        }
        log(newline);
    }
}

// prints the direction in word form
// @param d direction to print
void print_direction(direction d) {
    switch(d) {
        case up: 
            log((char*) "up\n");
            break;
        case down: 
            log((char* ) "down\n");
            break;
        case left: 
            log((char* ) "left\n");
            break;
        case right: 
            log((char* ) "right\n");
            break;
        default:
            log((char* ) "no direction\n");
    }
}

// prints a stack of MazeCells
// @param s stack of MazeCells to print
void print_stack(std::stack<MazeCell> s) {
    if(s.empty()) {
        log((char*)"\n");
        return;
    }
    MazeCell mc = s.top();
    s.pop();
    printMazeCell(mc);
    print_stack(s);
    s.push(mc);
}

// prints a stack of directions
// @param s stack of directions to print
void print_stack(std::stack<direction> s) {
    if(s.empty()) {
        log((char*)"\n");
        return;
    }
    direction d = s.top();
    s.pop();
    print_direction(d);
    print_stack(s);
    s.push(d);
}

// prints a queue of MazeCells
// @param q a queue of MazeCells to print
void print_queue(std::queue<MazeCell> q) {
    if(q.empty())
    {
        log((char*)"\n");
        return;
    }
    MazeCell mc = q.front();
    q.pop();
    printMazeCell(mc);
    print_queue(q);
    q.push(mc);
}

// prints a queue of directions
// @param q a queue of directions to print
void print_queue(std::queue<direction> q) {
    if(q.empty())
    {
        log((char*)"\n");
        return;
    }
    direction d = q.front();
    q.pop();
    print_direction(d);
    print_queue(q);
    q.push(d);
}

// prints a vector of MazeCells
void print_vector(std::vector<MazeCell> v) {
    for (int i = 0; i < v.size(); i++) {
        printMazeCell(v.at(i));
    }
}

// prints a vector of directions
void print_vector(std::vector<direction> v) {
for (int i = 0; i < v.size(); i++) {
        print_direction(v.at(i));
    }
}

// prints a vector of MoveInstructions
void print_vector(std::vector<MoveInstruction> v) {
    for (int i = 0; i < v.size(); i++) {
        log((char*)"start: ");
        printMazeCell(v.at(i).start);
        log((char*)"end: ");
        printMazeCell(v.at(i).end);
        print_direction(v.at(i).dir);
        log((char*)"\n");
    }
}