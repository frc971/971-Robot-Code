import os

os.environ['XLA_FLAGS'] = ' '.join([
    # Teach it where to find CUDA
    '--xla_gpu_cuda_data_dir=/usr/lib/cuda',
    # Use up to 20 cores
    '--xla_force_host_platform_device_count=2',
    # Dump XLA to /tmp/foo to aid debugging
    #'--xla_dump_to=/tmp/foo',
    #'--xla_gpu_enable_command_buffer='
])
os.environ['JAX_PLATFORMS'] = 'cpu'
#os.environ['XLA_PYTHON_CLIENT_PREALLOCATE'] = 'false'

import experience_buffer
import jax
import numpy
from jax.experimental import mesh_utils
from jax.sharding import Mesh, PartitionSpec, NamedSharding
import unittest


class TestExperienceBuffer(unittest.TestCase):

    def setUp(self):
        self.mesh = jax.sharding.Mesh(
            devices=mesh_utils.create_device_mesh(len(jax.devices())),
            axis_names=('batch', ),
        )

        self.sharding = jax.sharding.NamedSharding(self.mesh,
                                                   PartitionSpec('batch'))
        self.replicated_sharding = jax.sharding.NamedSharding(
            self.mesh, PartitionSpec())

        self.num_agents = 2
        self.sample_batch_size = 10
        self.length = 20

        self.buffer = experience_buffer.make_experience_buffer(
            self.num_agents, self.sample_batch_size, self.length)

    def test_shape(self):
        """Tests that the shapes coming out are right."""
        buffer_state = self.buffer.init({
            'key': jax.numpy.zeros((2, )),
        })

        for i in range(self.sample_batch_size // self.num_agents):
            buffer_state = self.buffer.add(
                buffer_state, {
                    'key':
                    jax.numpy.array(
                        [[i * 4, i * 4 + 1], [i * 4 + 2, i * 4 + 3]],
                        dtype=jax.numpy.float32)
                })

        rng = jax.random.key(0)

        for i in range(2):
            rng, sample_rng = jax.random.split(rng)
            batch = self.buffer.sample(buffer_state, sample_rng)
            self.assertEqual(batch.experience['key'].shape, (2, 5, 2))

    def test_randomness(self):
        """Tests that no sample is more or less likely."""
        rng = jax.random.key(0)

        # Adds an element to the buffer, and accumulates the sample.
        def loop(i, val):
            counts, buffer_state, rng = val

            buffer_state = self.buffer.add(
                buffer_state,
                {'key': jax.numpy.array([[-i], [i]], dtype=jax.numpy.int32)})

            rng, sample_rng = jax.random.split(rng)

            def do_count(counts):
                batch = self.buffer.sample(buffer_state, sample_rng)
                for a in range(self.num_agents):
                    for s in range(5):
                        sampled_agent = jax.numpy.abs(batch.experience['key'][
                            a, s, 0]) % (self.length // self.num_agents)
                        prior = counts[a, sampled_agent]
                        counts = counts.at[a, sampled_agent].set(prior + 1)

                return counts

            # If we are full, start randomly picking and counting.
            counts = jax.lax.cond(i >= self.length // self.num_agents,
                                  do_count, lambda counts: counts, counts)

            return counts, buffer_state, rng

        @jax.jit
        def doit(rng):
            buffer_state = self.buffer.init({
                'key':
                jax.numpy.zeros((1, ), dtype=jax.numpy.int32),
            })

            counts = numpy.zeros(
                (self.num_agents, self.length // self.num_agents))

            counts, buffer_state, rng = jax.lax.fori_loop(
                0,
                10000,
                loop,
                (jax.numpy.zeros(
                    (self.num_agents, self.length // self.num_agents)),
                 buffer_state, rng),
            )
            return counts

        # Do this all in jax to make it fast.  Many times speedup, including the JIT'ing.
        counts = numpy.array(doit(rng), dtype=numpy.int32)
        print(counts.min(), counts.max())

        # Make sure things are decently balanced.
        self.assertGreater(counts.min(), 4800)
        self.assertLess(counts.max(), 5200)


if __name__ == "__main__":
    unittest.main()
