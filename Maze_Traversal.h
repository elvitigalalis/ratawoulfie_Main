// #include <iostream> from motor.h
// #include <math.h> from drivetrain.h
#include <queue>
#include <stack>
#include <stdio.h>
// #include <stdlib.h> from API.h
#include "API.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

#ifndef MAZE_TRAVERSAL
#define MAZE_TRAVERSAL

/* ENUMERATIONS */
enum terrain {
  empty,         // 0: never travelled through for tremaux
  wall,          // 1: cannot travel through
  marked,        // 2: travelled once for tremaux
  double_marked, // 3: travelled twice for tremaux
  junction,      // 4: a traveled junction
  goal,          // 5: destination
  start          // 6: start cell
};

// direction mouse is facing
enum direction { up, down, left, right, no_direction };

// struct for cells
struct MazeCell {
  int row = 0;
  int col = 0;
  double g_score;
  double f_score;
  MazeCell(int r = 0, int c = 0, double gs = 9999, double fs = 9999)
      : row(r), col(c), g_score(gs), f_score(fs) {};
  bool operator!=(MazeCell mc) { return (mc.row == row) || (mc.col == col); }
};

struct Compare_F_Score {
  bool operator()(MazeCell *mc0, MazeCell *mc1) {
    return mc0->f_score > mc1->f_score;
  }
};

struct MoveInstruction {
  MazeCell start;
  MazeCell end;
  direction dir = no_direction;
};

terrain get_front_cell_terrain();
terrain get_behind_cell_terrain();
terrain get_left_cell_terrain();
terrain get_right_cell_terrain();
terrain get_current_cell_terrain();
terrain get_front_MazeCell_terrain();
terrain get_behind_MazeCell_terrain();
terrain get_left_MazeCell_terrain();
terrain get_right_MazeCell_terrain();
terrain get_current_cell_terrain();
terrain get_absolute_up_cell_terrain();
terrain get_absolute_down_cell_terrain();
terrain get_absolute_left_cell_terrain();
terrain get_absolute_right_cell_terrain();
terrain get_up_cell_terrain(int r, int c);
terrain get_down_cell_terrain(int r, int c);
terrain get_left_cell_terrain(int r, int c);
terrain get_right_cell_terrain(int r, int c);
MazeCell get_absolute_up_MazeCell(MazeCell mc);
MazeCell get_absolute_down_MazeCell(MazeCell mc);
MazeCell get_absolute_left_MazeCell(MazeCell mc);
MazeCell get_absolute_right_MazeCell(MazeCell mc);

bool set_adjacent_or_current_cell(terrain t, direction d = no_direction);
bool set_left_cell(terrain t);
bool set_right_cell(terrain t);
bool set_front_cell(terrain t);

void init_sample_maze(); // for testing purposes only
void init_maze();
void init_mouse(Drivetrain *dr);
void init_orientation_up(Drivetrain *dr);

// returns euclidean distance between two cartesian points (x0, y0) and (x1, y1)
double euclidean_distance(double x0, double y0, double x1, double y1);
// returns manhattan distance between two cartesian points
double manhattan_distance(double x0, double y0, double x1, double y1);
// returns the minimum double given two doubles
double min(double d0, double d1);
double conv_a_star_coord_to_maze_coord(int x);
double conv_maze_coord_to_a_star_coord(int x);
MazeCell conv_maze_coord_to_a_star_coord(int r, int c);

bool moveForward(Drivetrain *dr, int distance = 1, bool updateTerrain = true,
                 bool isUndo = false);
direction undoMove(Drivetrain *dr);
bool turnRight(Drivetrain *dr);
bool turnLeft(Drivetrain *dr);
bool moveUp(Drivetrain *dr, int distance = 1, bool updateTerrain = true,
            bool isUndo = false);
bool moveDown(Drivetrain *dr, int distance = 1, bool updateTerrain = true,
              bool isUndo = false);
bool moveLeft(Drivetrain *dr, int distance = 1, bool updateTerrain = true,
              bool isUndo = false);
bool moveRight(Drivetrain *dr, int distance = 1, bool updateTerrain = true,
               bool isUndo = false);
bool moveToAdjacentCell(Drivetrain *dr, MazeCell target,
                        bool updateTerrain = true, bool isUndo = false);

double euclidean_distance_heuristic();
double manhattan_distance_heuristic();
double euclidean_distance_heuristic(MazeCell mc0);
double manhattan_distance_heuristic(MazeCell mc0);
double euclidean_distance_heuristic(MazeCell mc0, MazeCell mc1);
double manhattan_distance_heuristic(MazeCell mc0, MazeCell mc1);
void tremaux_explore(Drivetrain *dr);
std::vector<MoveInstruction> a_star_traverse();
void travelPath(Drivetrain *dr, std::vector<MoveInstruction> moves,
                bool forward = true); // GOT RID OF DRIVETRAIN DR
MazeCell determine_goal_cell();

void print_maze();
void print_a_star_scores();
void printMazeCell(MazeCell mc);
void log(char *text);
void print_direction(direction d);
void print_stack(std::stack<MazeCell> s);
void print_stack(std::stack<direction> s);
void print_queue(std::queue<MazeCell> q);
void print_queue(std::queue<direction> q);
void print_vector(std::vector<MazeCell> v);
void print_vector(std::vector<direction> v);
void print_vector(std::vector<MoveInstruction> v);

#endif