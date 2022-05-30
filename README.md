# missile_simulation

https://github.com/osrf/gazebo/issues/354
clarification for the AddForce function's familly

      /// \brief Apply a force at the link's CoG for one time step.
      /// \param[in] _force Force expressed in the world frame.
      public: virtual void AddForce(const math::Vector3 &_force) = 0;

      /// \brief Apply a force at the link's CoG for one time step.
      /// \param[in] _force Force expressed in the link's frame (not its
      /// inertial frame).
      public: virtual void AddRelativeForce(const math::Vector3 &_force) = 0;

      /// \brief Apply a force at a given position for one time step.
      /// \param[in] _force Force expressed in the world frame.
      /// \param[in] _pos Position expressed in the world frame.
      public: virtual void AddForceAtWorldPosition(const math::Vector3 &_force,
                  const math::Vector3 &_pos) = 0;

      /// \brief Apply a force at a given position for one time step.
      /// Note that force and position are expressed in different frames.
      /// \param[in] _force Force expressed in the world frame.
      /// \param[in] _pos Position expressed in the link's frame (not its
      /// inertial frame).
      public: virtual void AddForceAtRelativePosition(
                  const math::Vector3 &_force,
                  const math::Vector3 &_relPos) = 0;