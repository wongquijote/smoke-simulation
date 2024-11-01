#ifndef CLOTH_H
#define CLOTH_H

#include <unordered_set>
#include <unordered_map>
#include <vector>

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "clothMesh.h"
#include "collision/collisionObject.h"
#include "spring.h"

using namespace CGL;
using namespace std;

enum e_orientation { HORIZONTAL = 0, VERTICAL = 1 };

struct ClothParameters {
  ClothParameters() {}
  ClothParameters(bool enable_structural_constraints,
                  bool enable_shearing_constraints,
                  bool enable_bending_constraints, double damping,
                  double density, double ks)
      : enable_structural_constraints(enable_structural_constraints),
        enable_shearing_constraints(enable_shearing_constraints),
        enable_bending_constraints(enable_bending_constraints),
        damping(damping), density(density), ks(ks) {}
  ~ClothParameters() {}

  // Global simulation parameters

  bool enable_structural_constraints;
  bool enable_shearing_constraints;
  bool enable_bending_constraints;

  double damping;

  // Mass-spring parameters
  double density;
  double ks;
};

struct Cloth {
  Cloth() {}
  Cloth(double width, double height, double depth, int num_width_points,
        int num_height_points, int num_depth_points, float thickness, bool show_grid);
  ~Cloth();

  void buildGrid();
  
  // We Changed this 
  Vector4D getMassColor(int x, int y, int z);
  void diffuse();
  void addSmokeSource(int x, int y, int z, int radius);
  void advect(double dt);
  void Cloth::project(vector<Vector4D>* prev_v_field, vector<Vector4D>* v_field);
  Vector4D trilinear_interpolate(double x, double y, double z);
  int IX(int x, int y, int z);
  Vector4D Cloth::getParticleProperty(int x, int y, int z, vector<Vector4D>* values);
  void Cloth::diffuse2(vector<Vector4D>* prev, vector<Vector4D>* curr, bool is_density);
  void Cloth::advect2(vector<Vector4D>* prev, vector<Vector4D>* curr, vector<Vector4D> curr_v, double dt, bool is_density);
  Vector4D Cloth::trilinear_interpolate2(vector<Vector4D>* values, double x, double y, double z);

  void simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                vector<Vector3D> external_accelerations,
                vector<CollisionObject *> *collision_objects);

  void reset();
  void buildClothMesh();

  void build_spatial_map();
  void self_collide(PointMass &pm, double simulation_steps);
  float hash_position(Vector3D pos);

  // Cloth properties
  double width;
  double height;
  double depth;
  int num_width_points;
  int num_height_points;
  int num_depth_points;
  double thickness;
  e_orientation orientation;
  // We Changed this
  bool show_grid;
  bool upward_smoke;
  Vector3D smoke_source;

  // Cloth components
  vector<PointMass> point_masses;
  vector<vector<int>> pinned;
  vector<Spring> springs;
  ClothMesh *clothMesh;
  // We Changed this
  // Current Velocity
  vector<Vector4D> velocity;
  // Previous Velocity along x axis
  vector<Vector4D> prev_velocity;
  // Current Density 
  vector<Vector4D> density;
  // Previous Density
  vector<Vector4D> prev_density;


  // Spatial hashing
  unordered_map<float, vector<PointMass *> *> map;
};

#endif /* CLOTH_H */
