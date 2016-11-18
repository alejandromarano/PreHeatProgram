typedef struct
  {
	float A0;          /**< The derived gain, A0 = Kp + Ki + Kd . */
    float A1;          /**< The derived gain, A1 = -Kp - 2Kd. */
    float A2;          /**< The derived gain, A2 = Kd . */
    float state[3];    /**< The state array of length 3. */
    float Kp;               /**< The proportional gain. */
    float Ki;               /**< The integral gain. */
    float Kd;               /**< The derivative gain. */
  } arm_pid_instance_f32;

static __INLINE float arm_pid_f32(
					arm_pid_instance_f32 * S,
					float in)
  {
    float out;

    /* y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]  */
    out = (S->A0 * in) +
      (S->A1 * S->state[0]) + (S->A2 * S->state[1]) + (S->state[2]);

    /* Update state */
    S->state[1] = S->state[0];
    S->state[0] = in;
    S->state[2] = out;

    /* return to application */
    return (out);

  }
