import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Manual sidebar structure for the Physical AI & Humanoid Robotics Book
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 Nervous System',
      items: [
        'modules/module-1-ros2/index',
        {
          type: 'category',
          label: 'Foundations',
          items: [
            'modules/module-1-ros2/foundations/what-is-ros2',
            'modules/module-1-ros2/foundations/ros1-vs-ros2',
            'modules/module-1-ros2/foundations/architecture-layers',
            'modules/module-1-ros2/foundations/installation-workspace',
          ],
        },
        {
          type: 'category',
          label: 'Communication',
          items: [
            'modules/module-1-ros2/communication/nodes-topics-services-actions',
          ],
        },
        {
          type: 'category',
          label: 'Integration',
          items: [
            'modules/module-1-ros2/integration/rclpy-integration',
            'modules/module-1-ros2/integration/command-pipelines',
          ],
        },
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin (Gazebo & Unity)',
      items: [
        'modules/module-2-digital-twin/index',
        {
          type: 'category',
          label: 'Foundations',
          items: [
            'modules/module-2-digital-twin/foundations/digital-twin-architecture',
            'modules/module-2-digital-twin/foundations/gazebo-setup-physics',
          ],
        },
        {
          type: 'category',
          label: 'Sensor Simulation',
          items: [
            'modules/module-2-digital-twin/sensor-simulation/lidar-simulation',
            'modules/module-2-digital-twin/sensor-simulation/rgbd-imu-simulation',
            'modules/module-2-digital-twin/sensor-simulation/validation-testing',
          ],
        },
        {
          type: 'category',
          label: 'Unity HRI',
          items: [
            'modules/module-2-digital-twin/unity-hri/unity-humanoid-integration',
          ],
        },
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'modules/module-3-ai-robot-brain/index',
        {
          type: 'category',
          label: 'Isaac Sim Setup',
          items: [
            'modules/module-3-ai-robot-brain/isaac-sim-setup/environment-setup',
          ],
        },
        {
          type: 'category',
          label: 'Perception Pipeline',
          items: [
            'modules/module-3-ai-robot-brain/perception-pipeline/isaac-ros-perception',
            'modules/module-3-ai-robot-brain/perception-pipeline/synthetic-data-generation',
          ],
        },
        {
          type: 'category',
          label: 'Navigation Integration',
          items: [
            'modules/module-3-ai-robot-brain/nav2-integration/navigation-stack',
            'modules/module-3-ai-robot-brain/nav2-integration/end-to-end-pipeline',
          ],
        },
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action Integration',
      items: [
        'modules/module-4-vla-integration/index',
        {
          type: 'category',
          label: 'Foundations',
          items: [
            'modules/module-4-vla-integration/foundations/vla-foundations',
          ],
        },
        {
          type: 'category',
          label: 'Voice-to-Action Pipeline',
          items: [
            'modules/module-4-vla-integration/voice-to-action/voice-pipeline',
          ],
        },
        {
          type: 'category',
          label: 'Capstone Design',
          items: [
            'modules/module-4-vla-integration/capstone/capstone-design',
          ],
        },
      ],
      collapsed: false,
    },
  ],
};

export default sidebars;
