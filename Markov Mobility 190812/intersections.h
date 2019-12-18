/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2006,2007 INRIA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */
#ifndef INTERSECTIONS_H
#define INTERSECTIONS_H

#include "ns3/object.h"
#include "ns3/nstime.h"
#include "ns3/event-id.h"
#include "ns3/rectangle.h"
#include "ns3/random-variable-stream.h"
#include "mobility-model.h"
#include "constant-velocity-helper.h"

namespace ns3 {


/**
 * \ingroup mobility
 * \brief 2D random walk mobility model.
 *
 * Each instance moves with a speed and direction choosen at random
 * with the user-provided random variables until
 * either a fixed distance has been walked or until a fixed amount
 * of time. If we hit one of the boundaries (specified by a rectangle),
 * of the model, we rebound on the boundary with a reflexive angle
 * and speed. This model is often identified as a brownian motion
 * model.
 */
class Intersections : public MobilityModel
{
public:
  /**
   * Register this type with the TypeId system.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  /** An enum representing the different working modes of this module. */
  enum Mode  {
    MODE_DISTANCE,
    MODE_TIME
  };

  Vector GetProbability();
  int GetDirection();

  int ch_direction;

  /* Markov value */
  int Num;
  int p1; // straight
  int p2; // right
  int p3; // left

private:
  /**
   * \brief Performs the rebound of the node if it reaches a boundary
   * \param timeLeft The remaining time of the walk
   */
  void Rebound (Time timeLeft);
  /**
   * Walk according to position and velocity, until distance is reached,
   * time is reached, or intersection with the bounding box
   */
  void DoWalk (Time timeLeft);
  /**
   * Perform initialization of the object before MobilityModel::DoInitialize ()
   */
  void DoInitializePrivate (void);
  virtual void DoDispose (void);
  virtual void DoInitialize (void);
  virtual Vector DoGetPosition (void) const;
  virtual void DoSetPosition (const Vector &position);
  virtual Vector DoGetVelocity (void) const;
  virtual uint8_t DoGetDirection (void) const;
  virtual void DoSetDirection (const uint8_t direction);
  virtual int64_t DoAssignStreams (int64_t);

  bool is_intersection(Vector speed, Vector nextPosition, Vector position);
  bool is_changeRN(Vector speed, Vector nextPosition, Vector position);
  void ChangeVelocity(Vector speed, Vector nextPosition, Vector position);
  void ChangeRN(Vector speed, Vector nextPosition, Vector position);
  int ChangedDirection();
  Vector InputVelocity();
  Vector DirVelocity(Vector velocity);

  double CalPercent(double a, double b);

  ConstantVelocityHelper m_helper; //!< helper for this object
  EventId m_event; //!< stored event ID
  Ptr<RandomVariableStream> m_speed; //!< rv for picking speed
  Rectangle m_bounds; //!< Bounds of the area to cruise
  uint32_t m_grid;
  uint32_t m_intersection;
  double m_distance;
  double m_delta;
  double m_deltaspeed;
  bool b_init = true;

  int n_r;
  int direction;

  bool b_st = false;
  bool b_rn = false;

};


} // namespace ns3

#endif /* RANDOM_WALK_2D_MOBILITY_MODEL_H */
