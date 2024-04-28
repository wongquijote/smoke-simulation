#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

// We Changed this: Added these
int timestep = 0;

Cloth::Cloth(double width, double height, double depth, int num_width_points,
             int num_height_points, int num_depth_points, float thickness, bool show_grid) {
  this->width = width;
  this->height = height;
  this->depth = depth;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->num_depth_points = num_depth_points;
  this->thickness = thickness;
  this->show_grid = show_grid;
  this->upward_smoke = true;

  buildGrid();
  // buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

// Helper functions //////////////////////////////

// Updates the density pointer values
void updateVector(Vector4D* value, Vector4D new_value, bool is_density) {
  if (is_density) {
    // Density needs to be between 0 and 1
    value->x = max(0.0, min(1.0, new_value.x));
    value->y = max(0.0, min(1.0, new_value.y));
    value->z = max(0.0, min(1.0, new_value.z));
    value->w = max(0.0, min(1.0, new_value.w));
  } else {
    value->x = new_value.x;
    value->y = new_value.y;
    value->z = new_value.z;
    value->w = new_value.w;
  }
}

//////////////////////////////////////////////////

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
    for (int k = 0; k < num_depth_points; k++) {
       for (int j = 0; j < num_height_points; j++) {
          for (int i = 0; i < num_width_points; i++) {
              double x = width * ((double)i / (double)num_width_points);
              double y = height * ((double)j / (double)num_height_points);
              double z = depth * ((double)k / (double)num_depth_points);
              Vector3D pos = Vector3D(x, y, z);
              PointMass* mass = new PointMass(pos, false);
              mass->prev_color = Vector4D(0.0);
              mass->color = Vector4D(0.0);
              mass->velocity = Vector3D(10.0, 25.0, 0.0);

              Vector4D* mass_density = new Vector4D();
              Vector4D* mass_prev_density = new Vector4D();
              Vector4D* mass_velocity = new Vector4D(10.0, 25.0, 0.0, 0.0);
              Vector4D* mass_prev_velocity = new Vector4D(10.0, 25.0, 0.0, 0.0);

              if (show_grid) {
                mass->prev_color = Vector4D(0.5);
                mass->color = Vector4D(0.5);

                updateVector(mass_prev_density, Vector4D(0.5), true);
                updateVector(mass_density, Vector4D(0.5), true);
              }

              this->density.push_back(*mass_density);
              this->prev_density.push_back(*mass_prev_density);
              this->velocity.push_back(*mass_velocity);
              this->prev_velocity.push_back(*mass_prev_velocity);
              this->point_masses.push_back(*mass);
              mass->smoke_source = false;
          }
      }
    }
}

// Calculates index of particle (x, y, z) 
int Cloth::IX(int x, int y, int z) {
  return x + y * this->num_width_points + z * (this->num_width_points * this->num_height_points);
}

/*
Adds a smoke source within out grid of pointmasees
*/
void Cloth::addSmokeSource(int x0, int y0, int z0, int radius) {
  if (show_grid) {
    return;
  }

  for (int i = (-1 * radius); i < radius; i++) {
    for (int j = (-1 * radius); j < radius; j++) {
      for (int k = (-1 * radius); k < radius; k++) {
        int x = x0 + i;
        int y = y0 + j;
        int z = z0 + k;
        if ((x >= 0) && (y >= 0) && (z >= 0) && (x < this->num_width_points)
        && (y < this->num_height_points) && (z < this->num_depth_points)) {
          int index = IX(x, y, z);
          PointMass* curr = &(this->point_masses[index]);
          curr->prev_color = Vector4D(1.0);
          curr->color = Vector4D(1.0);
          curr->smoke_source = true;

          Vector4D* curr_density = &(this->density[index]);
          Vector4D* curr_prev_density = &(this->prev_density[index]);
          updateVector(curr_density, Vector4D(1.0), true);
          updateVector(curr_prev_density, Vector4D(1.0), true);
        }
      }
    }
  }
  return;
}

/*
Helper method to check if there is a particle at object space (x, y, z) and returns its previous color
*/
Vector4D Cloth::getMassColor(int x, int y, int z) {
  if ((x >= 0) && (y >= 0) && (z >= 0) && (x < this->num_width_points)
  && (y < this->num_height_points) && (z < this->num_depth_points)) {
    int index = x + y * this->num_width_points + z * (this->num_width_points * this->num_depth_points);
    PointMass* curr = &(this->point_masses[index]);

    Vector4D* curr_density = &(this->prev_density[index]);
    return *curr_density;
  }
  return Vector4D(0.0);
}


// Returns the value corresponding to the coordinate (x, y, z) from values
Vector4D Cloth::getParticleProperty(int x, int y, int z, vector<Vector4D> values) {
  if ((x >= 0) && (y >= 0) && (z >= 0) && (x < this->num_width_points)
  && (y < this->num_height_points) && (z < this->num_depth_points)) {
    int index = IX(x, y, z);
    return values[index];
  }
  return Vector4D(0.0);
}

// We changed this (we added it)
/*
Diffuses the smoke intensity of all the particles in the scene
*/
void Cloth::diffuse() {
    timestep += 1;
    double diff = 0.8;
  for (int i = 0; i < this->point_masses.size(); i++) {
    PointMass *curr = &(this->point_masses[i]);

    if (curr->smoke_source) {
        if (timestep % 10 == 0) {
            curr->color = Vector4D(0.0);
            curr->prev_color = Vector4D(0.0);
            curr->smoke_source = false;
     }
      continue;
    }

    int x = i % this->num_width_points;
    int y = (i / this->num_width_points) % this->num_height_points;
    int z = i / (this->num_width_points * this->num_height_points);

    Vector4D front = getMassColor(x, y, z + 1);
    Vector4D back = getMassColor(x, y, z - 1);
    Vector4D left = getMassColor(x - 1, y, z);
    Vector4D right = getMassColor(x + 1, y, z);
    Vector4D above = getMassColor(x, y + 1, z);
    Vector4D below = getMassColor(x, y - 1, z);
    
    if (upward_smoke) {
      curr->color = (1.0 - diff) * curr->color + (diff / 8.0) * (front + back + left + right + above + below * 3.0);
    } else {
      curr->color = (1.0 - diff) * curr->color + (diff / 6.0) * (front + back + left + right + above + below);
    }
    
    curr->color.w = min(1.0, curr->color.w);
    curr->color.x = min(1.0, curr->color.x);
    curr->color.y = min(1.0, curr->color.y);
    curr->color.z = min(1.0, curr->color.z);
  }

  for (int i = 0; i < this->point_masses.size(); i++) {
    PointMass *curr = &(this->point_masses[i]);
    // update last color
    curr->prev_color = curr->color;
  }

  if (timestep % 3 == 0) {
      double t = timestep / 60.0;
      addSmokeSource((int)floor(25 + 10 * cos(t)), 5, (int)floor(25 + 10 * sin(t)), 1);
  }
}

void Cloth::diffuse2(vector<Vector4D>* prev, vector<Vector4D>* curr, bool is_density) {
  double diff = 0.8;
  int N_x = this->num_width_points;
  int N_y = this->num_height_points;
  int N_z = this->num_depth_points;

  for (int i = 0; i < (*prev).size(); i++) {
    int x_i = i % N_x;
    int y_i = (i / N_x) % N_y;
    int z_i = i / (N_x * N_y);

    Vector4D curr_value = (*prev)[IX(x_i, y_i, z_i)];
    Vector4D* new_value = &((*curr)[IX(x_i, y_i, z_i)]);

    Vector4D front = getParticleProperty(x_i, y_i, z_i + 1, *prev);
    Vector4D back  = getParticleProperty(x_i, y_i, z_i - 1, *prev);
    Vector4D left  = getParticleProperty(x_i - 1, y_i, z_i, *prev);
    Vector4D right = getParticleProperty(x_i + 1, y_i, z_i, *prev);
    Vector4D above = getParticleProperty(x_i, y_i + 1, z_i, *prev);
    Vector4D below = getParticleProperty(x_i, y_i - 1, z_i, *prev);

    Vector4D result = curr_value;

    if (upward_smoke) {
      result = (1.0 - diff) * curr_value + (diff / 8.0) * (front + back + left + right + above + below * 3.0);
    } else {
      result = (1.0 - diff) * curr_value + (diff / 6.0) * (front + back + left + right + above + below);
    }

    updateVector(new_value, result, is_density);
  }
  
  for (int i = 0; i < (*prev).size(); i++) {
    Vector4D* prev_value = &((*prev)[i]);
    Vector4D curr_value = (*curr)[i];

    // Update previous value
    updateVector(prev_value, curr_value, is_density);
  }

}

/*
given a 3 dimensional input, return the trilinear interpolation of the surrouding colors. handles out of indexing

use case is for backtracing velocity
*/
Vector4D Cloth::trilinear_interpolate(double x, double y, double z) {
  //edge case handling
  if (x < 0.5) {
    x = 0.0;
  } else if (x >= this->num_width_points - 1) {
    x = (double) this->num_width_points - 2;
  }
  if (y < 0.5) {
    y = 0.0;
  } else if (y >= this->num_height_points - 1) {
    y = (double) this->num_height_points - 2;
  }
  if (z < 0.5) {
    z = 0.0;
  } else if (z >= this->num_depth_points - 1) {
    z = (double) this->num_depth_points - 2;
  }

  //set up auxilliary variables. In order, these are coordinates, ratios, and colors
  int i0        = (int) floor(x);
  int j0        = (int) floor(y);
  int k0        = (int) floor(z);
  int i1        = i0 + 1;
  int j1        = j0 + 1;
  int k1        = k0 + 1;
  double s1     = x - (double) i0;
  double s0     = 1 - s1;
  double t1     = y - (double) j0;
  double t0     = 1 - t1;
  double u1     = z - (double) k0;
  double u0     = 1 - u1;
  Vector4D p000 = getMassColor(i0, j0, k0);
  Vector4D p001 = getMassColor(i0, j0, k1);
  Vector4D p010 = getMassColor(i0, j1, k0);
  Vector4D p011 = getMassColor(i0, j1, k1);
  Vector4D p100 = getMassColor(i1, j0, k0);
  Vector4D p101 = getMassColor(i1, j0, k1);
  Vector4D p110 = getMassColor(i1, j1, k0);
  Vector4D p111 = getMassColor(i1, j1, k1);

  //trilinear interpolate math
  Vector4D out = s0 * t0 * (u0 * p000 + u1 * p001) +
                 s0 * t1 * (u0 * p010 + u1 * p011) +
                 s1 * t0 * (u0 * p100 + u1 * p101) +
                 s1 * t1 * (u0 * p110 + u1 * p111);
  out.w = max(0.0, min(1.0, out.w));
  out.x = max(0.0, min(1.0, out.x));
  out.y = max(0.0, min(1.0, out.y));
  out.z = max(0.0, min(1.0, out.z));

  return out;
}

// trilinear interpolates values from values surrounding the coordinate (x, y, z)
Vector4D Cloth::trilinear_interpolate2(vector<Vector4D> values, double x, double y, double z) {
  //edge case handling
  if (x < 0.5) {
    x = 0.0;
  } else if (x >= this->num_width_points - 1) {
    x = (double) this->num_width_points - 2;
  }
  if (y < 0.5) {
    y = 0.0;
  } else if (y >= this->num_height_points - 1) {
    y = (double) this->num_height_points - 2;
  }
  if (z < 0.5) {
    z = 0.0;
  } else if (z >= this->num_depth_points - 1) {
    z = (double) this->num_depth_points - 2;
  }

  //set up auxilliary variables. In order, these are coordinates, ratios, and colors
  int i0        = (int) floor(x);
  int j0        = (int) floor(y);
  int k0        = (int) floor(z);
  int i1        = i0 + 1;
  int j1        = j0 + 1;
  int k1        = k0 + 1;
  double s1     = x - (double) i0;
  double s0     = 1 - s1;
  double t1     = y - (double) j0;
  double t0     = 1 - t1;
  double u1     = z - (double) k0;
  double u0     = 1 - u1;
  Vector4D p000 = getParticleProperty(i0, j0, k0, values);
  Vector4D p001 = getParticleProperty(i0, j0, k1, values);
  Vector4D p010 = getParticleProperty(i0, j1, k0, values);
  Vector4D p011 = getParticleProperty(i0, j1, k1, values);
  Vector4D p100 = getParticleProperty(i1, j0, k0, values);
  Vector4D p101 = getParticleProperty(i1, j0, k1, values);
  Vector4D p110 = getParticleProperty(i1, j1, k0, values);
  Vector4D p111 = getParticleProperty(i1, j1, k1, values);

  //trilinear interpolate math
  Vector4D out = s0 * t0 * (u0 * p000 + u1 * p001) +
                 s0 * t1 * (u0 * p010 + u1 * p011) +
                 s1 * t0 * (u0 * p100 + u1 * p101) +
                 s1 * t1 * (u0 * p110 + u1 * p111);
  // out.w = max(0.0, min(1.0, out.w));
  // out.x = max(0.0, min(1.0, out.x));
  // out.y = max(0.0, min(1.0, out.y));
  // out.z = max(0.0, min(1.0, out.z));

  return out;
}

void Cloth::advect(double dt) {
  for (int i = 0; i < this->point_masses.size(); i++) {
    PointMass *curr = &(this->point_masses[i]);
    if (curr->smoke_source) {
      continue;
    }

    int x_index = i % this->num_width_points;
    int y_index = (i / this->num_width_points) % this->num_height_points;
    int z_index = i / (this->num_width_points * this->num_height_points);

    //position if we went backwards by one timestep, assuming dt is 1 second
    double x = (double) x_index - curr->velocity.x * dt;
    double y = (double) y_index - curr->velocity.y * dt;
    double z = (double) z_index - curr->velocity.z * dt;

    curr->color = trilinear_interpolate(x, y, z);
    curr->color.w = min(1.0, curr->color.w);
    curr->color.x = min(1.0, curr->color.x);
    curr->color.y = min(1.0, curr->color.y);
    curr->color.z = min(1.0, curr->color.z);
    
  }
  for (int i = 0; i < this->point_masses.size(); i++) {
    PointMass *curr = &(this->point_masses[i]);
    // update last color
    curr->prev_color = curr->color;
  }
}

void Cloth::advect2(vector<Vector4D>* prev_d, vector<Vector4D>* curr_d, vector<Vector4D> curr_v, double dt, bool is_density) {
  int N_x = this->num_width_points;
  int N_y = this->num_height_points;
  int N_z = this->num_depth_points;

  for (int i = 0; i < (*prev_d).size(); i++) {
    int x_i = i % N_x;
    int y_i = (i / N_x) % N_y;
    int z_i = i / (N_x * N_y);

    Vector4D* curr_value = &((*curr_d)[IX(x_i, y_i, z_i)]); 
    Vector4D curr_velocity = curr_v[IX(x_i, y_i, z_i)];
    //position if we went backwards by one timestep, assuming dt is 1 second
    double x = (double) x_i - curr_velocity.x * dt;
    double y = (double) y_i - curr_velocity.y * dt;
    double z = (double) z_i - curr_velocity.z * dt;

    Vector4D res = trilinear_interpolate2(*prev_d, x, y, z);
    updateVector(curr_value, res, is_density);
  }

  for (int i = 0; i < (*prev_d).size(); i++) {
    Vector4D* prev_value = &((*prev_d)[i]);
    Vector4D curr_value = (*curr_d)[i];

    // Update previous value
    updateVector(prev_value, curr_value, is_density);
  }
}

void Cloth::project(vector<Vector4D>* prev_v_field, vector<Vector4D>* v_field) {
  int N_x = this->num_width_points;
  int N_y = this->num_height_points;
  int N_z = this->num_depth_points; 

  float h = 1.0 / ((float) (N_x));

  int x_i, y_i, z_i;
  Vector4D front, back, left, right, above, below;
  vector<Vector4D> proj, div;

  // Calculate gradient and store it in div
  for (int i = 0; i < (*v_field).size(); i++) {
    x_i = i % N_x;
    y_i = (i / N_x) % N_y;
    z_i = i / (N_x * N_y);
    
    front = getParticleProperty(x_i, y_i, z_i + 1, *v_field);
    back  = getParticleProperty(x_i, y_i, z_i - 1, *v_field);
    left  = getParticleProperty(x_i - 1, y_i, z_i, *v_field);
    right = getParticleProperty(x_i + 1, y_i, z_i, *v_field);
    above = getParticleProperty(x_i, y_i + 1, z_i, *v_field);
    below = getParticleProperty(x_i, y_i - 1, z_i, *v_field);

    Vector4D div_res = -0.5*h*((front.z - back.z) + (right.x - left.x) + (above.y - below.y)) * Vector4D(1.0);
    div.push_back(div_res);
    proj.push_back(Vector4D(0.0));
  }

  // Calculate projection
  for (int k = 0; k < 20; k++) {
    for (int i = 0; i < (*v_field).size(); i++) {
      x_i = i % N_x;
      y_i = (i / N_x) % N_y;
      z_i = i / (N_x * N_y);

      Vector4D curr_div = div[IX(x_i, y_i, z_i)];
      Vector4D* curr_p   = &(proj[IX(x_i, y_i, z_i)]);
      
      front = getParticleProperty(x_i, y_i, z_i + 1, proj);
      back  = getParticleProperty(x_i, y_i, z_i - 1, proj);
      left  = getParticleProperty(x_i - 1, y_i, z_i, proj);
      right = getParticleProperty(x_i + 1, y_i, z_i, proj);
      above = getParticleProperty(x_i, y_i + 1, z_i, proj);
      below = getParticleProperty(x_i, y_i - 1, z_i, proj);

      Vector4D proj_res = (1.0/6.0) * (curr_div + front + back + left + right + above + below);
      updateVector(curr_p, proj_res, false);
    }
  }

  // Update Velocity Field
  for (int i = 0; i < (*v_field).size(); i++) {
    x_i = i % N_x;
    y_i = (i / N_x) % N_y;
    z_i = i / (N_x * N_y);

    Vector4D* curr_v = &((*v_field)[IX(x_i, y_i, z_i)]);
    Vector4D* prev_v = &((*prev_v_field)[IX(x_i, y_i, z_i)]);

    front = getParticleProperty(x_i, y_i, z_i + 1, proj);
    back  = getParticleProperty(x_i, y_i, z_i - 1, proj);
    left  = getParticleProperty(x_i - 1, y_i, z_i, proj);
    right = getParticleProperty(x_i + 1, y_i, z_i, proj);
    above = getParticleProperty(x_i, y_i + 1, z_i, proj);
    below = getParticleProperty(x_i, y_i - 1, z_i, proj);

    Vector4D v_res = (*curr_v) - (0.5 * (1.0/h) * ((front.z - back.z) * Vector4D(0.0, 0.0, 1.0, 0.0)
                                                 + (right.x - left.x) * Vector4D(1.0, 0.0, 0.0, 0.0)
                                                 + (above.y - below.y)* Vector4D(0.0, 1.0, 0.0, 0.0)));
    updateVector(curr_v, v_res, false);
    updateVector(prev_v, *curr_v, false);
  }
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters* cp,
    vector<Vector3D> external_accelerations,
    vector<CollisionObject*>* collision_objects) {
    double delta_t = 1.0f / frames_per_sec / simulation_steps;
    
    if (!this->show_grid) {
      // velocity step
      // diffuse2(&(this->prev_velocity), &(this->velocity), false);
      // project(&(this->prev_velocity), &(this->velocity));
      // advect2(&(this->prev_velocity), &(this->velocity), this->prev_velocity, delta_t, false);
      // project(&(this->prev_velocity), &(this->velocity));

      // density step
      diffuse2(&(this->prev_density), &(this->density), true);
      advect2(&(this->prev_density), &(this->density), this->velocity, delta_t, true);
    }
}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
  for (int i = 0; i < this->point_masses.size(); i++) {
    PointMass* curr = &(this->point_masses[i]);
    float box = hash_position(curr->position);
    vector<PointMass*>* masses;
    if (this->map.count(box) == 0) {
      masses = new vector<PointMass*>;
      masses->push_back(curr);
      this->map[box] = masses;
    } else {
      masses = this->map[box];
      masses->push_back(curr);
    }
    this->map[box] = masses;
  }

}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
  float box = hash_position(pm.position);
  vector<PointMass*>* masses = this->map[box];
  Vector3D correction = Vector3D();
  int total = 0;
  for (int i = 0; i < masses->size(); i++) {
    PointMass* curr = (*masses)[i];
    if (&(*curr) == &pm) {
      // Same point mass
      continue;
    }
    double dist = (pm.position - curr->position).norm();
    if (dist < 2.0 * this->thickness) {
      correction += (pm.position - curr->position).unit() * (2.0 * this->thickness - dist);
      total += 1;
    }
  }

  if (total != 0) {
      correction = (correction / ((double)total)) / simulation_steps;
  }
  pm.position = pm.position + correction;
  
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
  double box_width = 3.0 * width / (double)num_width_points;
  double box_height = 3.0 * height / (double)num_height_points;
  double box_t = max(box_width, box_height);
  int x_hash = floor(pos.x / box_width);
  int y_hash = floor(pos.y / box_height);
  int z_hash = floor(pos.z / box_t);

  float res = x_hash + y_hash + z_hash;
  return res; 
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
