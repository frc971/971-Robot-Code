import flashbax
from flashbax.buffers.trajectory_buffer import TrajectoryBufferState, Experience, TrajectoryBufferSample
import jax
from jax.sharding import Mesh, PartitionSpec, NamedSharding
from jax.experimental import mesh_utils
from flax.typing import PRNGKey


def make_experience_buffer(num_agents, sample_batch_size, length):
    """Makes a random, sharded, fifo experience buffer."""
    mesh = jax.sharding.Mesh(
        devices=mesh_utils.create_device_mesh(len(jax.devices())),
        axis_names=('batch', ),
    )

    # Shard all the data along the agents axis.
    sharding = jax.sharding.NamedSharding(mesh, PartitionSpec('batch'))
    replicated_sharding = jax.sharding.NamedSharding(mesh, PartitionSpec())

    sample_batch_size = sample_batch_size // num_agents
    trajectory_buffer = flashbax.make_trajectory_buffer(
        max_length_time_axis=length // num_agents,
        min_length_time_axis=1,
        add_batch_size=num_agents,
        sample_batch_size=sample_batch_size,
        sample_sequence_length=1,
        period=1,
    )

    def add_fn(state: TrajectoryBufferState,
               batch: Experience) -> TrajectoryBufferState[Experience]:
        # Squeeze the data to match the shape desired by flashbax.
        batch_size, = flashbax.utils.get_tree_shape_prefix(batch, n_axes=1)
        expanded_batch = jax.tree.map(
            lambda x: x.reshape((batch_size, 1, *x.shape[1:])), batch)
        return trajectory_buffer.add(state, expanded_batch)

    def sample_fn(state: TrajectoryBufferState,
                  rng_key: PRNGKey) -> TrajectoryBufferSample[Experience]:
        batch_size, = flashbax.utils.get_tree_shape_prefix(state.experience,
                                                           n_axes=1)

        # Build up a RNG per actor so we can vmap the randomness.
        sample_keys = jax.device_put(jax.random.split(rng_key, num=batch_size),
                                     sharding)

        # Now, randomly select the indices to sample for a single agent.
        def single_item_indices(rng_key):
            return jax.random.randint(
                rng_key, (sample_batch_size, ), 0,
                jax.lax.select(state.is_full, length // num_agents,
                               state.current_index))

        # And do them all at once via vmap.
        item_indices = jax.vmap(single_item_indices)(sample_keys)

        # Actually sample them now, and vmap to do it for each agent.
        vmap_sample_item_indices = jax.vmap(
            lambda item_indices, x: x[item_indices])

        # And apply it to the tree.
        sampled_batch = jax.tree.map(
            lambda x: vmap_sample_item_indices(item_indices, x),
            state.experience)

        return flashbax.buffers.trajectory_buffer.TrajectoryBufferSample(
            experience=sampled_batch)

    def init_fn(experience: Experience):
        state = trajectory_buffer.init(experience)

        # Push each element of the tree out across the devices to shard it.
        sharded_experience = jax.tree_util.tree_map(
            lambda x: jax.device_put(x, sharding), state.experience)

        return flashbax.buffers.trajectory_buffer.TrajectoryBufferState(
            experience=sharded_experience,
            is_full=jax.device_put(state.is_full, replicated_sharding),
            current_index=jax.device_put(state.current_index,
                                         replicated_sharding),
        )

    return trajectory_buffer.replace(add=add_fn,
                                     sample=sample_fn,
                                     init=init_fn)
