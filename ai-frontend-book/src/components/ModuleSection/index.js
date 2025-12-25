import clsx from 'clsx';
import styles from './ModuleSection.module.css';
import ModuleCard from '../ModuleCard';

const moduleData = [
  {
    id: 1,
    title: 'ROS 2 Foundations',
    description: 'Learn ROS 2 — the nervous system of modern robots. Build nodes, topics, services, actions, publishers, subscribers, QoS, and real robot workflows.',
    link: '/docs/ros2-module/intro-to-ros2', // Adjust path as needed
    order: 1
  },
  {
    id: 2,
    title: 'Simulation & Digital Twins',
    description: 'Master simulation systems: Gazebo, Unity Robotics, Isaac Sim, and digital twin workflows for training and testing robots safely.',
    link: '/docs/digital-twin-module/intro', // Adjust path as needed
    order: 2
  },
  {
    id: 3,
    title: 'Hardware Foundations',
    description: 'Motors, actuators, torque control, IMUs, sensors, microcontrollers, and embedded systems required for humanoid robots.',
    link: '/docs/ai-robot-brain-module/isaac-sim-synthetic-data', // Adjust path as needed
    order: 3
  },
  {
    id: 4,
    title: 'VLA — Vision, Language, Action',
    description: 'Advanced robotics architecture combining perception models, large language models, and action planning systems.',
    link: '/docs/vla-module/intro', // Adjust path as needed
    order: 4
  },
  {
    id: 5,
    title: 'Sensor Systems',
    description: 'Depth cameras, LiDAR, sensor fusion, perception pipelines, and environmental awareness for autonomous robots.',
    link: '/docs/module-5-sensor-systems/fundamentals', // Adjust path as needed
    order: 5
  },
  {
    id: 6,
    title: 'Locomotion & Control',
    description: 'Humanoid walking, balance, gait planning, motion control, and stability mechanisms.',
    link: '/docs/module-6-locomotion/fundamentals', // Adjust path as needed
    order: 6
  }
];

const ModuleSection = () => {
  return (
    <section className={clsx('container', styles.moduleSection)}>
      <h2 className={styles.sectionTitle}>Explore All Modules</h2>
      <div className="row">
        {moduleData.map((module) => (
          <ModuleCard
            key={module.id}
            title={module.title}
            description={module.description}
            link={module.link}
          />
        ))}
      </div>
    </section>
  );
};

export default ModuleSection;