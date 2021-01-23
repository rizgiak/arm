#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "moveit_msgs/DisplayRobotState.h"
#include "moveit_msgs/ObjectColor.h"
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <vector>

#define PI (3.141592653589793)

using namespace std;

//global
vector<double> aTarget, aTarget2, aInit;
Eigen::Matrix<double, 6, 1> K, K2;
Eigen::Matrix<double, 6, 6> Ke, Ke2;
double dlt, T;
Eigen::Matrix<double, 6, 6> jEt, jEt2, jEi, jEi2; //JE, sensei
Eigen::Vector3d pTarget, pTarget2, pInit, pInit2; //PTar, PE, sensei
Eigen::Matrix3d rTarget, rTarget2, rInit, rInit2; //RTar, RE, sensei

double deg2rad(double degree)
{
  return degree * (PI / 180);
}

double rad2deg(double rad)
{
  return rad * (180 / PI);
}

Eigen::Vector3d rot2angle(Eigen::Matrix3d rotation)
{
  const double sg = 1;
  const double ep = 0;

  Eigen::Vector3d result;
  result << 0, 0, 0;

  // cout << "rotation:" << rotation << endl;

  Eigen::Vector3d lll;
  lll << rotation.coeff(2, 1) - rotation.coeff(1, 2),
      rotation.coeff(0, 2) - rotation.coeff(2, 0),
      rotation.coeff(1, 0) - rotation.coeff(0, 1);

  // cout << "lll:" << lll << endl;

  double abslll = lll.norm();
  // cout << "abslll:" << abslll << endl;

  if (abslll > ep)
  {
    result = atan2(abslll, rotation.trace() - 1) / abslll * lll;
  }
  else if (abslll <= ep && rotation.trace() >= 3 - ep)
  {
    result << 0, 0, 0;
  }
  else if (abslll <= ep && rotation.trace() <= -1 + ep)
  {
    Eigen::Vector3d nn, nnn;
    if (rotation.coeff(1, 0) >= 0 && rotation.coeff(2, 1) >= 0 && rotation.coeff(0, 2) >= 0)
    {
      nnn << sqrt((rotation.coeff(0, 0) + 1) / 2),
          sqrt((rotation.coeff(1, 1) + 1) / 2),
          sqrt((rotation.coeff(2, 2) + 1) / 2);
      nn = sg * nnn;
    }
    else if (rotation.coeff(1, 0) >= 0 && rotation.coeff(2, 1) < 0 && rotation.coeff(0, 2) <= 0)
    {
      nnn << sqrt((rotation.coeff(0, 0) + 1) / 2),
          sqrt((rotation.coeff(1, 1) + 1) / 2),
          -sqrt((rotation.coeff(2, 2) + 1) / 2);
      nn = sg * nnn;
    }
    else if (rotation.coeff(1, 0) < 0 && rotation.coeff(2, 1) <= 0 && rotation.coeff(0, 2) >= 0)
    {
      nnn << sqrt((rotation.coeff(0, 0) + 1) / 2),
          -sqrt((rotation.coeff(1, 1) + 1) / 2),
          sqrt((rotation.coeff(2, 2) + 1) / 2);
      nn = sg * nnn;
    }
    else if (rotation.coeff(1, 0) <= 0 && rotation.coeff(2, 1) >= 0 && rotation.coeff(0, 2) < 0)
    {
      nnn << -sqrt((rotation.coeff(0, 0) + 1) / 2),
          sqrt((rotation.coeff(1, 1) + 1) / 2),
          sqrt((rotation.coeff(2, 2) + 1) / 2);
      nn = sg * nnn;
    }
    else
    {
      cout << "undefined if rot2angle" << endl;
    }
    result = PI * nn;
  }
  else
  {
    cout << "undefined rot2angle" << endl;
  }
  // cout << "rot2angle:" << result << endl;
  return result;
}

Eigen::Matrix3d RotV(Eigen::RowVector3d vect, Eigen::RowVector3d angle)
{
  Eigen::Matrix3d a;
  return a;
}

Eigen::Matrix3d RotV(Eigen::RowVector3d vect)
{
  Eigen::Matrix3d rotationMatrix;
  Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  Eigen::RowVector3d w;
  double normResult = vect.norm();
  //cout << "vect:" << vect << endl
  //     << "normResult:" << normResult << endl;
  if (normResult == 0)
  {
    w << 0, 0, 0;
  }
  else
  {
    w = vect / normResult;
  }
  //cout << "w:" << w << endl;
  Eigen::Matrix3d generate;
  generate << 0, -w[2], w[1], w[2], 0, -w[0], -w[1], w[0], 0;

  rotationMatrix = w.transpose() * w + cos(normResult) * (identity - w.transpose() * w) - sin(normResult) * generate;
  // cout << "rotMatrix: " << endl
  //      << rotationMatrix << endl
  //      << endl;
  return rotationMatrix;
}

// Eigen::Matrix3f RotV(Eigen::RowVector3f vect)
// {
//   Eigen::AngleAxisf rollAngle(vect[2], Eigen::Vector3f::UnitZ());
//   Eigen::AngleAxisf yawAngle(vect[1], Eigen::Vector3f::UnitY());
//   Eigen::AngleAxisf pitchAngle(vect[0], Eigen::Vector3f::UnitX());
//   Eigen::Quaternion<float> q = rollAngle * yawAngle * pitchAngle;
//   Eigen::Matrix3f rotationMatrix = q.matrix();
//   cout << "rotMatrix: " << rotationMatrix << endl;
//   return rotationMatrix;
// }

void ForwardKinematics2nd(vector<double> angle, Eigen::Vector3d &PE, Eigen::Matrix3d &RE, Eigen::Matrix<double, 6, 6> &JE)
{
  Eigen::Vector3d L1, B2, L2, B3, L3, L4, L5, L6;
  Eigen::RowVector3d ph1, ph2, ph3, ph4, ph5, ph6;
  Eigen::RowVector3d t_ph1, t_ph2, t_ph3, t_ph4, t_ph5, t_ph6;
  Eigen::Matrix3d R1, R2, R3, R4, R5, R6;
  Eigen::Matrix<Eigen::Vector3d, 2, 6> result;

  L1 << 0, 0, 100;
  B2 << 0, 30, 0;
  L2 << 0, 0, 100;
  B3 << 0, -20, 0;
  L3 << 0, 0, 80;
  L4 << 0, 0, 50;
  L5 << 0, 0, 60;
  L6 << 0, 0, 50;

  ph1 << 0, 0, 1;
  ph2 << 0, 1, 0;
  ph3 << 0, 1, 0;
  ph4 << 0, 0, 1;
  ph5 << 0, 1, 0;
  ph6 << 0, 0, 1;

  R1 = RotV(ph1 * angle[0]).transpose();
  R2 = RotV(ph2 * angle[1]).transpose();
  R3 = RotV(ph3 * angle[2]).transpose();
  R4 = RotV(ph4 * angle[3]).transpose();
  R5 = RotV(ph5 * angle[4]).transpose();
  R6 = RotV(ph6 * angle[5]).transpose();

  Eigen::Vector3d P1, PB2, P2, PB3, P3, P4, P5;
  Eigen::Vector3d P0 = Eigen::Vector3d::Zero();
  P1 = R1 * L1;
  PB2 = R1 * (L1 + R2 * B2);
  P2 = R1 * (L1 + R2 * (B2 + L2));
  PB3 = R1 * (L1 + R2 * ((B2 + L2) + R3 * B3));
  P3 = R1 * (L1 + R2 * ((B2 + L2) + R3 * (B3 + L3)));
  P4 = R1 * (L1 + R2 * ((B2 + L2) + R3 * ((B3 + L3) + R4 * (L4))));
  P5 = R1 * (L1 + R2 * ((B2 + L2) + R3 * ((B3 + L3) + R4 * (L4 + R5 * (L5)))));
  //PE = R1 * (L1 + R2 * ((B2 + L2) + R3 * ((B3 + L3) + R4 * (L4 + R5 * (L5 + R6 * L6)))));
  PE = P3;
  //PL = [ P0, P1, PB2, P2, PB3, P3, P4, P5, PE ];
  //RE = R1 * R2 * R3 * R4 * R5 * R6;
  RE = R1 * R2 * R3;

  // ph create new memory

  t_ph1 = ph1.transpose();
  t_ph2 = R1 * ph2.transpose();
  t_ph3 = R1 * R2 * ph3.transpose();
  t_ph4 = R1 * R2 * R3 * ph4.transpose();
  t_ph5 = R1 * R2 * R3 * R4 * ph5.transpose();
  t_ph6 = R1 * R2 * R3 * R4 * R5 * ph6.transpose();

  vector<Eigen::Vector3d> p, r;
  p.push_back(t_ph1.cross(PE - P0));
  p.push_back(t_ph2.cross(PE - P1));
  p.push_back(t_ph3.cross(PE - P2));
  p.push_back(t_ph4.cross(PE - P3));
  p.push_back(t_ph5.cross(PE - P4));
  p.push_back(t_ph6.cross(PE - P5));

  r.push_back(t_ph1.transpose());
  r.push_back(t_ph2.transpose());
  r.push_back(t_ph3.transpose());
  r.push_back(t_ph4.transpose());
  r.push_back(t_ph5.transpose());
  r.push_back(t_ph6.transpose());

  for (int i = 0; i < 3; i++)
  {
    JE(i, 0) = p[i](0);
    JE(i, 1) = p[i](1);
    JE(i, 2) = p[i](2);
    JE(i, 3) = r[i](0);
    JE(i, 4) = r[i](1);
    JE(i, 5) = r[i](2);
  }

  for (int i = 3; i < p.size(); i++)
  {
    JE(i, 0) = 0;
    JE(i, 1) = 0;
    JE(i, 2) = 0;
    JE(i, 3) = 0;
    JE(i, 4) = 0;
    JE(i, 5) = 0;
  }

  JE.transposeInPlace();
}

void ForwardKinematics(vector<double> angle, Eigen::Vector3d &PE, Eigen::Matrix3d &RE, Eigen::Matrix<double, 6, 6> &JE)
{
  Eigen::Vector3d L1, B2, L2, B3, L3, L4, L5, L6;
  Eigen::RowVector3d ph1, ph2, ph3, ph4, ph5, ph6;
  Eigen::RowVector3d t_ph1, t_ph2, t_ph3, t_ph4, t_ph5, t_ph6;
  Eigen::Matrix3d R1, R2, R3, R4, R5, R6;
  Eigen::Matrix<Eigen::Vector3d, 2, 6> result;

  L1 << 0, 0, 100;
  B2 << 0, 30, 0;
  L2 << 0, 0, 100;
  B3 << 0, -20, 0;
  L3 << 0, 0, 80;
  L4 << 0, 0, 50;
  L5 << 0, 0, 60;
  L6 << 0, 0, 50;

  ph1 << 0, 0, 1;
  ph2 << 0, 1, 0;
  ph3 << 0, 1, 0;
  ph4 << 0, 0, 1;
  ph5 << 0, 1, 0;
  ph6 << 0, 0, 1;

  R1 = RotV(ph1 * angle[0]).transpose();
  R2 = RotV(ph2 * angle[1]).transpose();
  R3 = RotV(ph3 * angle[2]).transpose();
  R4 = RotV(ph4 * angle[3]).transpose();
  R5 = RotV(ph5 * angle[4]).transpose();
  R6 = RotV(ph6 * angle[5]).transpose();

  Eigen::Vector3d P1, PB2, P2, PB3, P3, P4, P5;
  Eigen::Vector3d P0 = Eigen::Vector3d::Zero();
  P1 = R1 * L1;
  PB2 = R1 * (L1 + R2 * B2);
  P2 = R1 * (L1 + R2 * (B2 + L2));
  PB3 = R1 * (L1 + R2 * ((B2 + L2) + R3 * B3));
  P3 = R1 * (L1 + R2 * ((B2 + L2) + R3 * (B3 + L3)));
  P4 = R1 * (L1 + R2 * ((B2 + L2) + R3 * ((B3 + L3) + R4 * (L4))));
  P5 = R1 * (L1 + R2 * ((B2 + L2) + R3 * ((B3 + L3) + R4 * (L4 + R5 * (L5)))));
  PE = R1 * (L1 + R2 * ((B2 + L2) + R3 * ((B3 + L3) + R4 * (L4 + R5 * (L5 + R6 * L6)))));
  //PL = [ P0, P1, PB2, P2, PB3, P3, P4, P5, PE ];
  RE = R1 * R2 * R3 * R4 * R5 * R6;

  // ph create new memory

  t_ph1 = ph1.transpose();
  t_ph2 = R1 * ph2.transpose();
  t_ph3 = R1 * R2 * ph3.transpose();
  t_ph4 = R1 * R2 * R3 * ph4.transpose();
  t_ph5 = R1 * R2 * R3 * R4 * ph5.transpose();
  t_ph6 = R1 * R2 * R3 * R4 * R5 * ph6.transpose();

  vector<Eigen::Vector3d> p, r;
  p.push_back(t_ph1.cross(PE - P0));
  p.push_back(t_ph2.cross(PE - P1));
  p.push_back(t_ph3.cross(PE - P2));
  p.push_back(t_ph4.cross(PE - P3));
  p.push_back(t_ph5.cross(PE - P4));
  p.push_back(t_ph6.cross(PE - P5));

  r.push_back(t_ph1.transpose());
  r.push_back(t_ph2.transpose());
  r.push_back(t_ph3.transpose());
  r.push_back(t_ph4.transpose());
  r.push_back(t_ph5.transpose());
  r.push_back(t_ph6.transpose());

  for (int i = 0; i < p.size(); i++)
  {
    JE(i, 0) = p[i](0);
    JE(i, 1) = p[i](1);
    JE(i, 2) = p[i](2);
    JE(i, 3) = r[i](0);
    JE(i, 4) = r[i](1);
    JE(i, 5) = r[i](2);
  }

  JE.transposeInPlace();
}

void Initialize()
{
  aTarget.push_back(deg2rad(10)); //yaw
  aTarget.push_back(deg2rad(10));
  aTarget.push_back(deg2rad(-5));
  aTarget.push_back(deg2rad(30));
  aTarget.push_back(deg2rad(60));
  aTarget.push_back(deg2rad(0)); //yaw

  aTarget2.push_back(deg2rad(10)); //yaw
  aTarget2.push_back(deg2rad(-20));
  aTarget2.push_back(deg2rad(-15));
  aTarget2.push_back(deg2rad(30));
  aTarget2.push_back(deg2rad(0));
  aTarget2.push_back(deg2rad(0)); //yaw

  aInit.push_back(deg2rad(0));
  aInit.push_back(deg2rad(0));
  aInit.push_back(deg2rad(0));
  aInit.push_back(deg2rad(0));
  aInit.push_back(deg2rad(0));
  aInit.push_back(deg2rad(0));

  K << 1, 1, 1, 100, 100, 100;
  Ke = K.asDiagonal();

  K2 << 1, 1, 1, 100, 100, 100;
  Ke2 = K2.asDiagonal();

  dlt = T = 1; //theta 0

  ForwardKinematics(aTarget, pTarget, rTarget, jEt);
  ForwardKinematics2nd(aTarget2, pTarget2, rTarget2, jEt2);

  ForwardKinematics(aInit, pInit, rInit, jEi);
  ForwardKinematics2nd(aInit, pInit2, rInit2, jEi2); //sensei
  cout << "Init: " << pInit << endl;
  cout << "Target 1st: " << endl
       << pTarget << endl;
  cout << "Target 2nd: " << endl
       << pTarget2 << endl;
}
void InverseKinematics(double &vN, double &V)
{
  Eigen::Vector3d pError, rError;
  pError = pTarget - pInit;
  rError = rot2angle(rTarget * rInit.transpose());

  // cout << "jEi:" << endl
  //      << jEi << endl;

  // cout << "Ke:" << endl
  //      << Ke << endl;

  Eigen::Matrix<double, 6, 1> e, dqk;
  Eigen::Matrix<double, 6, 1> tau;

  e << pError(0), pError(1), pError(2), rError(0), rError(1), rError(2);
  tau = jEi.transpose() * Ke * e;
  // here
  // cout << "e:" << endl
  //      << e << endl;

  // cout << "tau:" << endl
  //      << tau << endl;

  Eigen::Matrix<double, 6, 6> A, D;
  A = jEi.transpose() * Ke * jEi;
  V = 0.5 * e.transpose() * Ke * e;

  // cout << "A:" << endl
  //      << A << endl;

  // cout << "V:" << V << endl;

  D = T * (A + (0.5 * V + dlt) * Eigen::Matrix<double, 6, 6>::Identity());

  // cout << "D:" << endl
  //      << D << endl;
  dqk = D.inverse() * tau;
  // cout << "dqk:" << endl
  //      << dqk << endl;
  Eigen::Matrix<double, 6, 1> qk;
  qk << aInit[0], aInit[1], aInit[2], aInit[3], aInit[4], aInit[5];
  qk = qk + dqk * T;

  // cout << "qk:" << endl
  //      << qk << endl;

  aInit.clear();
  aInit.push_back(qk(0));
  aInit.push_back(qk(1));
  aInit.push_back(qk(2));
  aInit.push_back(qk(3));
  aInit.push_back(qk(4));
  aInit.push_back(qk(5));

  ForwardKinematics(aInit, pInit, rInit, jEi);
  cout << "target_iteration:" << endl
       << pInit << endl;
  pError = pTarget - pInit;
  rError = rot2angle(rTarget * rInit.transpose());
  e << pError(0), pError(1), pError(2), rError(0), rError(1), rError(2);
  // cout << "e:" << endl
  //      << e << endl;
  vN = 0.5 * e.transpose() * Ke * e;
  cout << "vN:" << vN << " V:" << V << endl;
}

void InverseKinematics2(double &vN, double &vN2, double &V, double &V2, double &Te)
{
  Eigen::Vector3d pError, rError;
  pError = pTarget - pInit;
  rError = rot2angle(rTarget * rInit.transpose());

  // cout << "jEi:" << endl
  //      << jEi << endl;

  // cout << "Ke:" << endl
  //      << Ke << endl;

  Eigen::Matrix<double, 6, 1> e1, e2, dqk;
  Eigen::Matrix<double, 6, 1> tau1, tau2, tau;

  e1 << pError(0), pError(1), pError(2), rError(0), rError(1), rError(2);
  tau1 = jEi.transpose() * Ke * e1;

  pError = pTarget2 - pInit2;                        //sensei
  rError = rot2angle(rTarget2 * rInit2.transpose()); //sensei
  e2 << pError(0), pError(1), pError(2), rError(0), rError(1), rError(2);
  tau2 = jEi2.transpose() * Ke2 * e2; //sensei

  tau = tau1 + tau2;
  // here
  // cout << "e:" << endl
  //      << e << endl;

  // cout << "tau:" << endl
  //      << tau << endl;

  Eigen::Matrix<double, 6, 6> A, D;
  A = (jEi.transpose() * Ke * jEi) + (jEi2.transpose() * Ke2 * jEi2); //sensei
  V = 0.5 * e1.transpose() * Ke * e1;
  V2 = 0.5 * e2.transpose() * Ke2 * Te * e2;
  // cout << "A:" << endl
  //      << A << endl;

  // cout << "V:" << V << endl;

  D = T * (A + (0.5 * (V + V2) + dlt) * Eigen::Matrix<double, 6, 6>::Identity());

  // cout << "D:" << endl
  //      << D << endl;
  dqk = D.inverse() * tau;
  // cout << "dqk:" << endl
  //      << dqk << endl;
  Eigen::Matrix<double, 6, 1> qk;
  qk << aInit[0], aInit[1], aInit[2], aInit[3], aInit[4], aInit[5];
  qk = qk + dqk * T;

  // cout << "qk:" << endl
  //      << qk << endl;

  aInit.clear();
  aInit.push_back(qk(0));
  aInit.push_back(qk(1));
  aInit.push_back(qk(2));
  aInit.push_back(qk(3));
  aInit.push_back(qk(4));
  aInit.push_back(qk(5));

  ForwardKinematics(aInit, pInit, rInit, jEi);
  cout << "target_iteration 1st:" << endl
       << pInit << endl;

  //just show 2nd target
  //Eigen::Vector3d p_temp;
  //Eigen::Matrix3d r_temp;
  //Eigen::Matrix<double, 6, 6> j_temp;

  //ForwardKinematics2nd(aInit, p_temp, r_temp, j_temp);
  ForwardKinematics2nd(aInit, pInit2, rInit2, jEi2);
  cout << "target_iteration 2nd:" << endl
       << pInit2 << endl;

  pError = pTarget - pInit;
  rError = rot2angle(rTarget * rInit.transpose());
  e1 << pError(0), pError(1), pError(2), rError(0), rError(1), rError(2);

  pError = pTarget2 - pInit2;
  rError = rot2angle(rTarget2 * rInit2.transpose());
  e2 << pError(0), pError(1), pError(2), rError(0), rError(1), rError(2);

  // cout << "e:" << endl
  //      << e << endl;
  vN = 0.5 * e1.transpose() * Ke * e1;
  vN2 = 0.5 * e2.transpose() * Ke2 * Te * e2;

  if ((vN + vN2) >= 0.99 * (V + V2))
  {
    Te = Te - 0.25 * 1; //Tk+1 = Tk -  === 0.75
  }

  if (Te < 0)
  {
    Te = 0;
  }

  cout << "vN:" << vN << " V:" << V << " vN2:" << vN2 << " V2:" << V2 << " Te:" << Te << endl;
}

sensor_msgs::JointState JointController(string name, double pos, double velocity, double effort)
{
  sensor_msgs::JointState msg;

  msg.name.push_back(name);
  msg.position.push_back(pos);
  msg.velocity.push_back(velocity);
  msg.effort.push_back(effort);

  return msg;
}

moveit_msgs::ObjectColor Coloring(string id, std_msgs::ColorRGBA color)
{
  moveit_msgs::ObjectColor col;
  col.id = id;
  col.color = color;
  return col;
}

moveit_msgs::DisplayRobotState DisplayRobot(string name, double pos, int target)
{
  moveit_msgs::DisplayRobotState msg;
  std_msgs::ColorRGBA col, colAlpha;

  colAlpha.r = 0;
  colAlpha.g = 0;
  colAlpha.b = 0;
  colAlpha.a = 0;

  switch (target)
  {
  case 1:
    col.r = 0.1;
    col.g = 0.65;
    col.b = 0.02;
    col.a = 0.75;
    break;
  case 2:
    col.r = 0.1;
    col.g = 0.02;
    col.b = 0.62;
    col.a = 0.75;
    break;

  default:
    col.r = 0.1;
    col.g = 0.65;
    col.b = 0.02;
    col.a = 0.75;
    break;
  }

  msg.state.joint_state = JointController(name, pos, 1, 0);
  msg.highlight_links.push_back(Coloring("l0", col));
  msg.highlight_links.push_back(Coloring("l1", col));
  msg.highlight_links.push_back(Coloring("b2", col));
  msg.highlight_links.push_back(Coloring("l2", col));
  msg.highlight_links.push_back(Coloring("b3", col));
  msg.highlight_links.push_back(Coloring("l3", col));
  if (target == 1)
  {
    msg.highlight_links.push_back(Coloring("l4", col));
    msg.highlight_links.push_back(Coloring("l5", col));
    msg.highlight_links.push_back(Coloring("l6", col));
  }
  else
  {
    msg.highlight_links.push_back(Coloring("l4", colAlpha));
    msg.highlight_links.push_back(Coloring("l5", colAlpha));
    msg.highlight_links.push_back(Coloring("l6", colAlpha));
  }
  return msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_position_controller");

  ros::NodeHandle n;

  ros::Publisher fake_controller = n.advertise<sensor_msgs::JointState>("move_group/fake_controller_joint_states", 1000);
  ros::Publisher init_controller = n.advertise<moveit_msgs::DisplayRobotState>("robot_state_init", 1000);
  ros::Publisher first_target_controller = n.advertise<moveit_msgs::DisplayRobotState>("robot_state_first_target", 1000);
  ros::Publisher second_target_controller = n.advertise<moveit_msgs::DisplayRobotState>("robot_state_second_target", 1000);

  ros::Rate loop_rate(0.5);

  moveit_msgs::DisplayRobotState init;

  Initialize();

  cout << "aInit:" << rad2deg(aInit[0]) << ", " << rad2deg(aInit[1]) << ", " << rad2deg(aInit[2]) << ", " << rad2deg(aInit[3]) << ", " << rad2deg(aInit[4]) << ", " << rad2deg(aInit[5]) << endl;
  double vN, vN2, V, V2, Te = 1; //Tk = 1
  int count = 0;
  while (ros::ok())
  {
    //Message Publish ---
    {
      first_target_controller.publish(DisplayRobot("l0_l1_joint", aTarget[0], 1));
      first_target_controller.publish(DisplayRobot("l1_b2_joint", aTarget[1], 1));
      first_target_controller.publish(DisplayRobot("l2_b3_joint", aTarget[2], 1));
      first_target_controller.publish(DisplayRobot("l3_l4_joint", aTarget[3], 1));
      first_target_controller.publish(DisplayRobot("l4_l5_joint", aTarget[4], 1));
      first_target_controller.publish(DisplayRobot("l5_l6_joint", aTarget[5], 1));

      second_target_controller.publish(DisplayRobot("l0_l1_joint", aTarget2[0], 2));
      second_target_controller.publish(DisplayRobot("l1_b2_joint", aTarget2[1], 2));
      second_target_controller.publish(DisplayRobot("l2_b3_joint", aTarget2[2], 2));
      second_target_controller.publish(DisplayRobot("l3_l4_joint", aTarget2[3], 2));
      second_target_controller.publish(DisplayRobot("l4_l5_joint", aTarget2[4], 2));
      second_target_controller.publish(DisplayRobot("l5_l6_joint", aTarget2[5], 2));

      fake_controller.publish(JointController("l0_l1_joint", aInit[0], 1, 0));
      fake_controller.publish(JointController("l1_b2_joint", aInit[1], 1, 0));
      fake_controller.publish(JointController("l2_b3_joint", aInit[2], 1, 0));
      fake_controller.publish(JointController("l3_l4_joint", aInit[3], 1, 0));
      fake_controller.publish(JointController("l4_l5_joint", aInit[4], 1, 0));
      fake_controller.publish(JointController("l5_l6_joint", aInit[5], 1, 0));
    }
    

    cout << "Iteration " << count << endl;
    cout << "result: " << rad2deg(aInit[0]) << ", " << rad2deg(aInit[1]) << ", " << rad2deg(aInit[2]) << ", " << rad2deg(aInit[3]) << ", "
         << rad2deg(aInit[4]) << ", " << rad2deg(aInit[5]) << endl;
    //InverseKinematics(vN, V);
    InverseKinematics2(vN, vN2, V, V2, Te);
    ros::spinOnce();
    loop_rate.sleep();
    count++;
    if (vN + vN2 > 0.999 * (V + V2) && vN2 == 0)
      break;

    //V2 = vN2;
  }
  return 0;
}
