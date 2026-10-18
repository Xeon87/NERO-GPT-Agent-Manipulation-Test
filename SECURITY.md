# Security and Real-Robot Safety

The public release is a simulation stack. Treat any future real-robot port as a separate safety validation effort.

- Never let an LLM bypass hardware E-stop, torque/force limits, collision monitors, workspace limits, or supervised commissioning.
- Keep automatic recovery bounded and independently interruptible.
- Do not put API keys in screenshots, logs, commits, or bug reports.
- Review downloaded third-party robot packages before running scripts with write access to ROS configuration files.
