// SPDX-License-Identifier: LGPL-3.0
#include <sim2b/functions/bullet.h>

#include <b3RobotSimulatorClientAPI_NoGUI.h>


struct sim2b_bullet
{
    b3RobotSimulatorClientAPI_NoGUI *sim;
    int robot_id;
};


void sim2b_bullet_configure(struct sim2b_bullet_nbx *b)
{
    assert(b);
    assert(b->urdf);

    b->bullet = new sim2b_bullet;
    b->bullet->sim = new b3RobotSimulatorClientAPI_NoGUI();
    b3RobotSimulatorClientAPI_NoGUI *sim = b->bullet->sim;

    // Setup the simulation
	sim->connect(eCONNECT_DIRECT);
	sim->resetSimulation();
	sim->setGravity(btVector3(b->grav[0], b->grav[1], b->grav[2]));

    // Load the robot from a URDF file
    b3RobotSimulatorLoadUrdfFileArgs urdf_args;
    // Prevent the base from free-fall
    urdf_args.m_forceOverrideFixedBase = true;
    // Use inertia and collision shapes as specified in URDF
    urdf_args.m_flags = URDF_USE_INERTIA_FROM_FILE | URDF_USE_SELF_COLLISION;
    int robot_id = sim->loadURDF(b->urdf, urdf_args);
    b->bullet->robot_id = robot_id;

    // Check that number of joints are consistent
    int nr_joints = sim->getNumJoints(robot_id);
    int nr_non_fixed_joints = 0;
    for (int i = 0; i < nr_joints; i++) {
        b3JointInfo info;
        sim->getJointInfo(robot_id, i, &info);

        if (info.m_jointType != JointType::eFixedType) nr_non_fixed_joints++;
    }
    assert(nr_non_fixed_joints == b->nr_joints);

    // Set initial configuration (if given)
    for (int i = 0; i < nr_joints; i++) {
        if (b->jnt_pos_init) {
            sim->resetJointState(robot_id, i, b->jnt_pos_init[i]);
        }
    }
}


void sim2b_bullet_start(struct sim2b_bullet_nbx *b)
{
    assert(b);
    assert(b->bullet);
    assert(b->bullet->sim);

    // Unlock brakes (as required for torque control in Bullet)
    for (int i = 0; i < b->nr_joints; i++) {
        b3RobotSimulatorJointMotorArgs c(CONTROL_MODE_VELOCITY);
        c.m_maxTorqueValue = 0.0;

        b->bullet->sim->setJointMotorControl(b->bullet->robot_id, i, c);
    }
}


void sim2b_bullet_step(struct sim2b_bullet_nbx *b)
{
    assert(b);
    assert(b->bullet);
    assert(b->bullet->sim);
    assert(b->ctrl_mode);
    assert(b->jnt_pos_cmd);
    assert(b->jnt_vel_cmd);
    assert(b->jnt_eff_cmd);
    assert(b->jnt_pos_msr);
    assert(b->jnt_vel_msr);

    b3RobotSimulatorClientAPI_NoGUI *sim = b->bullet->sim;

    // Command the robot
    for (int i = 0; i < b->nr_joints; i++) {
        b3RobotSimulatorJointMotorArgs c(0);

        switch (*b->ctrl_mode) {
            case 0:
                assert(b->jnt_pos_cmd);
                assert(b->jnt_eff_max);

                c.m_controlMode = CONTROL_MODE_POSITION_VELOCITY_PD;
                c.m_targetPosition = b->jnt_pos_cmd[i];
                c.m_maxTorqueValue = b->jnt_eff_max[i];
            break;

            case 1:
                assert(b->jnt_vel_cmd);
                assert(b->jnt_eff_max);

                c.m_controlMode = CONTROL_MODE_VELOCITY;
                c.m_targetVelocity = b->jnt_vel_cmd[i];
                c.m_maxTorqueValue = b->jnt_eff_max[i];
            break;

            case 2:
                c.m_controlMode = CONTROL_MODE_TORQUE;
                c.m_maxTorqueValue = b->jnt_eff_cmd[i];
            break;
        }

        sim->setJointMotorControl(b->bullet->robot_id, i, c);
    }

    sim->stepSimulation();

    // Measure the robot's state
    for (int i = 0; i < b->nr_joints; i++) {
        b3JointSensorState s;
        sim->getJointState(b->bullet->robot_id, i, &s);

        b->jnt_pos_msr[i] = s.m_jointPosition;
        b->jnt_vel_msr[i] = s.m_jointVelocity;
    }
}


void sim2b_bullet_cleanup(struct sim2b_bullet_nbx *b)
{
    assert(b);
    assert(b->bullet);

    delete b->bullet->sim;
    delete b->bullet;
}
