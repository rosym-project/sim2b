return block
{
    name = "sim_bullet",
    license = "LGPL3",
    meta_data = "Bullet based simulator '",

    port_cache = true,

    configurations = {
       { name = "nr_joints", type_name = "int", min = 1, max = 1, doc = "Number of joints" },
       { name = "gravity", type_name = "double", min = 0, max = 3, doc = "Acceleration vector for simulated gravity (XYZ [m/s^2])" },
       { name = "jnt_pos_init", type_name = "double", min = 1, doc = "Initial joint positions ([m] or [rad] depending on joint type)" },
       { name = "urdf", type_name = "char", min = 1, doc = "Path to URDF file containing a description of the robot that should be simulated" },
    },

    ports = {
       -- Inputs
        { name = "ctrl_mode", in_type_name = "int", in_data_len = 1, doc = "Control mode; 0: VELOCITY_PD, 1: VELOCITY (jnt_vel_cmd port), 2: TORQUE (jnt_eff_cmd port)" },
        { name = "jnt_pos_cmd", in_type_name = "double", doc = "Commanded joint positions ([m] or [rad] depending on joint type)" },
        { name = "jnt_vel_cmd", in_type_name = "double", doc = "Commanded joint velocities ([m/s] or [rad/s] depending on joint type)" },
        { name = "jnt_eff_cmd", in_type_name = "double", doc = "Commanded joint torques ([N] or [Nm] depending on joint type)" },
        { name = "jnt_eff_max", in_type_name = "double", doc = "Maximal joint torques ([N] or [Nm] depending on joint type)" },
        -- Outputs
        { name = "jnt_pos_msr", out_type_name = "double", doc = "Measured joint positions ([m] or [rad] depending on joint type)" },
        { name = "jnt_vel_msr", out_type_name = "double", doc = "Measured joint velocities ([m/s] or [rad/s] depending on joint type)" },
    },

    operations = { step=true }
}
