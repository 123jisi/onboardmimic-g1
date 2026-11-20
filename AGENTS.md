# Source and deployment

- Develop and review source changes on the host. Deployment directories on the robot contain a deployed copy and machine-local data.
- Compare the actual SHA-256 of deployed source files before reporting source parity.
- Keep vendor binaries, models, engines, virtual environments, credentials, user motions, calibrations and other runtime data outside Git.
- Run the offline regression tests for source changes. Record offline, idle-system and physical motion acceptance separately.
- Readiness checks and source synchronization do not authorize starting any robot motion.
