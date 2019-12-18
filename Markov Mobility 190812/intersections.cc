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
#include "intersections.h"
#include "ns3/enum.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include "ns3/uinteger.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include <cmath>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("Intersections");

NS_OBJECT_ENSURE_REGISTERED (Intersections);

TypeId
Intersections::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::Intersections")
    .SetParent<MobilityModel> ()
    .SetGroupName ("Mobility")
    .AddConstructor<Intersections> ()
    .AddAttribute ("Grid", "number of lines",
                   UintegerValue (2),
                   MakeUintegerAccessor (&Intersections::m_grid),
                   MakeUintegerChecker<uint32_t> ())
     .AddAttribute ("Intersection", "number of intersections",
                    UintegerValue (1),
                    MakeUintegerAccessor (&Intersections::m_intersection),
                    MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("Distance", "Distance between intersections",
                   DoubleValue (500.0),
                   MakeDoubleAccessor (&Intersections::m_distance),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Speed", "A random variable used to pick the speed (m/s).",
                   StringValue ("ns3::UniformRandomVariable[Min=16|Max=18]"),
                   MakePointerAccessor (&Intersections::m_speed),
                   MakePointerChecker<RandomVariableStream> ())
     .AddAttribute ("Bounds",
                    "Bounds of the area to cruise.",
                    RectangleValue (Rectangle (0.0, 100.0, 0.0, 100.0)),
                    MakeRectangleAccessor (&Intersections::m_bounds),
                    MakeRectangleChecker ())
     .AddAttribute ("DeltaSpeed", "Delta value of speeds",
                    DoubleValue (5.556),
                    MakeDoubleAccessor (&Intersections::m_delta),
                    MakeDoubleChecker<double> ());
  return tid;
}

void
Intersections::DoInitialize (void)
{
  Vector position = m_helper.GetCurrentPosition ();
  double Rx = position.x - (m_distance / 2 - (3 * m_grid + 1.5)) - ((int)((double)position.x/m_distance) * m_distance);
  double Ry = position.y - (m_distance / 2 - (3 * m_grid + 1.5)) - ((int)((double)position.y/m_distance) * m_distance);
  if(Rx > 0 && (double)Rx/3 - (int)((double)Rx/3) == 0){
    if(Rx <= 3 * m_grid){
      direction = 0;
    }else{
      direction = 1;
    }
  }else{
    if(Ry <= 3 * m_grid){
      direction = 2;
    }else{
      direction = 3;
    }
  }

  m_deltaspeed = UniformRandomVariable ().GetValue (-m_delta, m_delta);

  if(m_grid == 1){
    if(CalPercent(position.x, m_distance) < m_distance/2 - 3 * m_grid && CalPercent(position.y, m_distance) > m_distance/2) b_st = true;
    else if(CalPercent(position.x, m_distance) > m_distance/2 + 3 * m_grid && CalPercent(position.y, m_distance) < m_distance/2) b_st = true;
    else if(CalPercent(position.x, m_distance) < m_distance/2 && CalPercent(position.y, m_distance) < m_distance/2 - 3 * m_grid) b_st = true;
    else if(CalPercent(position.x, m_distance) > m_distance/2 && CalPercent(position.y, m_distance) > m_distance/2 + 3 * m_grid) b_st = true;
    else b_st = false;
  } else {
    b_st = false;
  }

  /*Markov initialization */
  Num = 100;
  p1 = UniformRandomVariable().GetValue(0, 100);
  p2 = UniformRandomVariable().GetValue(0, 100 - p1);
  p3 = 100 - p1 - p2;
  n_r = (int) UniformRandomVariable().GetValue(0, m_grid-0.01);

  double a = UniformRandomVariable ().GetValue (0, 100);
  if((p1 >= 5 && a < p1) || (p1 < 5 && a < 5)){
    ch_direction = 0; 
  }else if((p2 >= 5 && a < p1 + p2) || (p2 < 5 && a < p1 + 5)){
    ch_direction = 1;
  }else {
    ch_direction = 2;
  }


  DoInitializePrivate ();
  MobilityModel::DoInitialize ();
}

void
Intersections::DoInitializePrivate (void)
{
  m_helper.Update ();
  double speed = m_speed->GetValue ();
  int i = direction;
  Vector vector;
  if(b_init){
    vector = Vector (((i * 2 - 5) * (i / 2)) * (speed + m_deltaspeed),
                    ((1 - (i / 2)) * (1 - 2 * i)) * (speed + m_deltaspeed), 0);
    b_init = false;
  }else{
    vector = InputVelocity();
  }

  m_helper.SetVelocity (vector);
  m_helper.Unpause ();

  DoWalk (Seconds(0.1));
}

Vector Intersections::InputVelocity(){
  Vector velocity = m_helper.GetVelocity ();
  double speed = m_speed->GetValue () + m_deltaspeed;
  int x1 = speed * 1000;
  int x2 = x1 / 10;
  speed = speed - (x1-x2*10)*0.001 + ch_direction*0.001;
  //Vector velocity(0, 0, 0);
  if(velocity.x == 0){
    if(velocity.y > 0)
      velocity.y = speed;
    else
      velocity.y = speed * (-1);
  } else {
    if(velocity.x > 0)
      velocity.x = speed;
    else
      velocity.x = speed * (-1);
  }
  //std::cout << "a : " << velocity << "\n";
  return velocity;
}

Vector Intersections::DirVelocity(Vector velocity){
  double speed = m_speed->GetValue () + m_deltaspeed;
  int x1 = speed * 1000;
  int x2 = x1 / 10;
  speed = speed - (x1-x2*10)*0.001 + ch_direction*0.001;
  //Vector velocity(0, 0, 0);
  if(velocity.x == 0){
    if(velocity.y > 0)
      velocity.y = speed;
    else
      velocity.y = speed * (-1);
  } else {
    if(velocity.x > 0)
      velocity.x = speed;
    else
      velocity.x = speed * (-1);
  }
  //std::cout << "a : " << velocity << "\n";
  return velocity;
}

void
Intersections::DoWalk (Time delayLeft)
{
  Vector position = m_helper.GetCurrentPosition ();
  Vector speed = m_helper.GetVelocity ();
  Vector nextPosition = position;
  nextPosition.x += speed.x * delayLeft.GetSeconds ();
  nextPosition.y += speed.y * delayLeft.GetSeconds ();
  m_event.Cancel ();
  if (m_bounds.IsInside (nextPosition))
  {
    if(b_st){
      if(is_intersection(speed, nextPosition, position)){
        m_event = Simulator::Schedule (delayLeft, &Intersections::ChangeVelocity, this, speed, nextPosition, position);
        //ChangeVelocity(speed, nextPosition, position);
      } else{
        m_event = Simulator::Schedule (delayLeft, &Intersections::DoInitializePrivate, this);
      }
    } else {
      if(is_changeRN(speed, nextPosition, position)){
        m_event = Simulator::Schedule (delayLeft, &Intersections::ChangeRN, this, speed, nextPosition, position);
        //ChangeRN(speed, nextPosition, position);
      } else{
        m_event = Simulator::Schedule (delayLeft, &Intersections::DoInitializePrivate, this);
      }
    }
  }
  else
  {
    nextPosition = m_bounds.CalculateIntersection (position, speed);
    m_event = Simulator::Schedule (delayLeft, &Intersections::Rebound, this, delayLeft);
  }
  NotifyCourseChange ();
}

void
Intersections::Rebound (Time delayLeft)
{
  m_helper.UpdateWithBounds (m_bounds);
  Vector position = m_helper.GetCurrentPosition ();
  Vector speed = m_helper.GetVelocity ();
  b_st = true;
  switch (m_bounds.GetClosestSide (position))
    {
    case Rectangle::RIGHT:
      if(ch_direction == 1){
        position.y -= CalPercent(position.y, m_distance) - (m_distance/2 - (3*m_grid-1.5));
      } else if(ch_direction == 2){
        position.y -= CalPercent(position.y, m_distance) - (m_distance/2 - 1.5);
      } else {
        position.y -= 3 * m_grid;
      }
      position.x -= 1;
      break;
    case Rectangle::LEFT:
      if(ch_direction == 1){
        position.y += m_distance/2 + (3*m_grid-1.5) - CalPercent(position.y, m_distance);
      } else if(ch_direction == 2){
        position.y += m_distance/2 + 1.5 - CalPercent(position.y, m_distance);
      } else {
        position.y += 3 * m_grid;
      }
      position.x += 1;
      break;
    case Rectangle::TOP:
      if(ch_direction == 1){
        position.x += m_distance/2+(3*m_grid-1.5) - CalPercent(position.x, m_distance);
      } else if(ch_direction == 2){
        position.x += m_distance/2+1.5 - CalPercent(position.x, m_distance);
      } else {
        position.x += 3 * m_grid;
      }
      position.y -= 1;
      break;
    case Rectangle::BOTTOM:
      if(ch_direction == 1){
        position.x -= CalPercent(position.x, m_distance) - (m_distance/2-(3*m_grid-1.5));
      } else if(ch_direction == 2){
        position.x -= CalPercent(position.x, m_distance) - (m_distance/2-1.5);
      } else {
        position.x -= 3 * m_grid;
      }
      position.y += 1;
      break;
    }

  speed.x = speed.x * (-1);
  speed.y = speed.y * (-1);
  m_helper.SetPosition (position);
  m_helper.SetVelocity (speed);
  m_helper.Unpause ();
  DoWalk (delayLeft);
}

void Intersections::ChangeVelocity(Vector speed, Vector nextPosition, Vector position){
  b_st = false;
  Num++;
  if(ch_direction == 0) p1++;
  else if(ch_direction == 1) p2++;
  else p3++;

  if(speed.x == 0){
    if(speed.y < 0){
      switch (ch_direction)
      {
      case 1: // up -> right
        position.x += m_distance/2+(3*m_grid-1.5) - CalPercent(nextPosition.y, m_distance);
        position.y -= CalPercent(position.y, m_distance) - (m_distance/2 + (3*m_grid-1.5));
        speed.x = -1 * speed.y;
        speed.y = 0;
        break;
      case 2: // up -> left
        position.x -= m_distance/2-1.5 - CalPercent(nextPosition.y, m_distance);
        position.y -= CalPercent(position.y, m_distance) - (m_distance/2-1.5);
        speed.x = 1 * speed.y;
        speed.y = 0;
        break;      
      default:
        break;
      }
    } else if(speed.y > 0){
      switch (ch_direction)
      {
      case 1: // down -> left
        position.x -= CalPercent(nextPosition.y, m_distance) - (m_distance/2-(3*m_grid-1.5));
        position.y += m_distance/2-(3*m_grid-1.5) - CalPercent(position.y, m_distance);
        speed.x = -1 * speed.y;
        speed.y = 0;
        break;
      case 2: // down -> right
        position.x += CalPercent(nextPosition.y, m_distance) - (m_distance/2+1.5);
        position.y += m_distance/2+1.5 - CalPercent(position.y, m_distance);
        speed.x = speed.y;
        speed.y = 0;
        break;      
      default:
        break;
      }
    }
  }else if(speed.y == 0){
    if(speed.x < 0){
      switch (ch_direction)
      {
      case 1: // left -> up
        position.y -= m_distance/2+(3*m_grid-1.5) - CalPercent(nextPosition.x, m_distance);
        position.x -= CalPercent(position.x, m_distance) - (m_distance/2+(3*m_grid-1.5));
        speed.y = 1 * speed.x;
        speed.x = 0;
        break;
      case 2: // left -> down
        position.y += m_distance/2-1.5 - CalPercent(nextPosition.x, m_distance);
        position.x -= CalPercent(position.x, m_distance) - (m_distance/2-1.5);
        speed.y = -1 * speed.x;
        speed.x = 0;
        break;      
      default:
        break;
      }
    } else if(speed.x > 0){
      switch (ch_direction)
      {
      case 1: // right -> down
        position.y += CalPercent(nextPosition.x, m_distance) - (m_distance/2-(3*m_grid-1.5));
        position.x += (m_distance/2-(3*m_grid-1.5)) - CalPercent(position.x, m_distance);
        speed.y = speed.x;
        speed.x = 0;
        break;
      case 2: // right -> up
        position.y -= CalPercent(nextPosition.x, m_distance) - (m_distance/2+1.5);
        position.x += (m_distance/2+1.5) - CalPercent(position.x, m_distance);
        speed.y = -1*speed.x;
        speed.x = 0;
        break;      
      default:
        break;
      }
    }
  }
  ch_direction = ChangedDirection();
  if(speed.x == 0) speed.y = DirVelocity(speed).y;
  else speed.x = DirVelocity(speed).x;

  m_helper.SetPosition (position);
  m_helper.SetVelocity (speed);
  m_helper.Unpause ();
  DoWalk (Seconds(0.1));
}

void Intersections::ChangeRN(Vector speed, Vector nextPosition, Vector position){
  b_st = true;

  if(speed.x == 0){
    if(speed.y < 0){
      if(ch_direction == 1 && (CalPercent(position.x, m_distance) < m_distance/2+(3*m_grid-1.5))){
        position.x += m_distance/2+(3*m_grid-1.5) - CalPercent(position.x, m_distance);
      } else if(ch_direction == 2 && (CalPercent(position.x, m_distance) > m_distance/2+1.5)){
        position.x -= CalPercent(position.x, m_distance) - (m_distance/2+1.5);
      }
    } else {
      if(ch_direction == 1 && (CalPercent(position.x, m_distance) > m_distance/2-(3*m_grid-1.5))){
        position.x -= CalPercent(position.x, m_distance) - (m_distance/2-(3*m_grid-1.5));
      } else if(ch_direction == 2 && (CalPercent(position.x, m_distance) < (m_distance/2-1.5))){
        position.x += (m_distance/2-1.5) - CalPercent(position.x, m_distance);
      }
    }    
  }else if(speed.y == 0){
    if(speed.x < 0){
      if(ch_direction == 1 && (CalPercent(position.y, m_distance) > (m_distance/2-(3*m_grid-1.5)))){
        position.y -= CalPercent(position.y, m_distance) - (m_distance/2-(3*m_grid-1.5));
      } else if(ch_direction == 2 && (CalPercent(position.y, m_distance) < m_distance/2-1.5)){
        position.y += m_distance/2-1.5 - CalPercent(position.y, m_distance);
      }
    } else {
      if(ch_direction == 1 && (CalPercent(position.y, m_distance) < m_distance/2+(3*m_grid-1.5))){
        position.y +=  m_distance/2+(3*m_grid-1.5) - CalPercent(position.y, m_distance);
      } else if(ch_direction == 2 && (CalPercent(position.y, m_distance) > m_distance/2+1.5)){
        position.y -= CalPercent(position.y, m_distance) - (m_distance/2+1.5);
      }
    }
  }

  m_helper.SetPosition (position);
  m_helper.SetVelocity (speed);
  m_helper.Unpause ();
  DoWalk (Seconds(0.1));
}

int Intersections::ChangedDirection(){
  double tmp = UniformRandomVariable().GetValue(0, 100);
  double pp[3];
  pp[0] = (double)100 * p1/Num;
  pp[1] = (double)100 * p2/Num;
  pp[2] = (double)100 * p3/Num;

  if((pp[0] >= 5 && tmp < pp[0]) || (pp[0] < 5 && tmp < 5)){
    ch_direction = 0; 
  }else if((pp[1] >= 5 && tmp < pp[0] + pp[1]) || (pp[1] < 5 && tmp < pp[0] + 5)){
    ch_direction = 1;
  }else {
    ch_direction = 2;
  }
  return ch_direction;
}

bool Intersections::is_intersection(Vector speed, Vector nextPosition, Vector position){
  if(speed.x == 0){
    if(ch_direction == 0){ // straight
      if(speed.y < 0 && m_distance/2 >= CalPercent(nextPosition.y, m_distance))
        return true;
      else if(speed.y > 0 && m_distance/2 <= CalPercent(nextPosition.y, m_distance))
        return true;
    } else if(ch_direction == 1){ // right
      if(speed.y < 0 && m_distance/2+(3*m_grid-1.5) >= CalPercent(nextPosition.y, m_distance))
        return true;
      else if(speed.y > 0 && m_distance/2-(3*m_grid-1.5) <= CalPercent(nextPosition.y, m_distance))
        return true;
    } else if(ch_direction == 2){ // left
      if(speed.y < 0 && m_distance/2-1.5 >= CalPercent(nextPosition.y, m_distance))
        return true;
      else if(speed.y > 0 && m_distance/2+1.5 <= CalPercent(nextPosition.y, m_distance))
        return true;
    }
  }else if(speed.y == 0){
    if(ch_direction == 0){ // straight
      if(speed.x < 0 && m_distance/2 >= CalPercent(nextPosition.x, m_distance))
        return true;
      else if(speed.x > 0 && m_distance/2 <= CalPercent(nextPosition.x, m_distance))
        return true;
    } else if(ch_direction == 1){ // right
      if(speed.x < 0 && m_distance/2 + (3*m_grid-1.5) >= CalPercent(nextPosition.x, m_distance))
        return true;
      else if(speed.x > 0 && m_distance/2-(3*m_grid-1.5) <= CalPercent(nextPosition.x, m_distance))
        return true;
    } else if(ch_direction == 2){ // left
      if(speed.x < 0 && m_distance/2-1.5 >= CalPercent(nextPosition.x, m_distance))
        return true;
      else if(speed.x > 0 && m_distance/2+1.5 <= CalPercent(nextPosition.x, m_distance))
        return true;
    }
  }
  return false;
}

bool Intersections::is_changeRN(Vector speed, Vector nextPosition, Vector position){
  if(speed.x == 0){
    if(speed.y < 0 && (CalPercent(position.y, m_distance) - CalPercent(nextPosition.y, m_distance) < 0)){
      return true;
    } else if(speed.y > 0 && (CalPercent(position.y, m_distance) - CalPercent(nextPosition.y, m_distance) > 0)){
      return true;
    }
  } else if(speed.y == 0){
    if(speed.x < 0 && (CalPercent(position.x, m_distance) - CalPercent(nextPosition.x, m_distance) < 0)){
      return true;
    } else if(speed.x > 0 && (CalPercent(position.x, m_distance) - CalPercent(nextPosition.x, m_distance) > 0)){
      return true;
    }
  }
  return false;
}

double Intersections::CalPercent(double a, double b){
  double c = a;
  for(;c >= 0;c-=b){}
  c += b;
  return c;
}

Vector
Intersections::GetProbability ()
{
  Vector p;
  p.x = 100 * (double) p1/Num;
  p.y = 100 * (double) p2/Num;
  p.z = 100 * (double) p3/Num;
  return p;
}

int Intersections::GetDirection(){
  return ch_direction;
}

void
Intersections::DoDispose (void)
{
  // chain up
  MobilityModel::DoDispose ();
}
Vector
Intersections::DoGetPosition (void) const
{
  m_helper.UpdateWithBounds (m_bounds);
  return m_helper.GetCurrentPosition ();
}
void
Intersections::DoSetPosition (const Vector &position)
{
  NS_ASSERT (m_bounds.IsInside (position));
  m_helper.SetPosition (position);
  Simulator::Remove (m_event);
  m_event = Simulator::ScheduleNow (&Intersections::DoInitializePrivate, this);
}
Vector
Intersections::DoGetVelocity (void) const
{
  return m_helper.GetVelocity ();
}
uint8_t
Intersections::DoGetDirection (void) const
{
  return 0;
}
void
Intersections::DoSetDirection (const uint8_t direction)
{
}
int64_t
Intersections::DoAssignStreams (int64_t stream)
{
  m_speed->SetStream (stream);
  //m_direction->SetStream (stream + 1);
  return 2;
}


} // namespace ns3
