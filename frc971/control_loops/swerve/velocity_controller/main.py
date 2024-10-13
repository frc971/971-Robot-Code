import os

# Setup XLA first.
os.environ['XLA_FLAGS'] = ' '.join([
    # Teach it where to find CUDA
    '--xla_gpu_cuda_data_dir=/usr/lib/cuda',
    # Use up to 20 cores
    #'--xla_force_host_platform_device_count=6',
    # Dump XLA to /tmp/foo to aid debugging
    #'--xla_dump_to=/tmp/foo',
    #'--xla_gpu_enable_command_buffer='
    # Dump sharding
    #"--xla_dump_to=/tmp/foo",
    #"--xla_dump_hlo_pass_re=spmd|propagation"
])
os.environ['JAX_PLATFORMS'] = 'cuda,cpu'
os.environ['XLA_PYTHON_CLIENT_PREALLOCATE'] = 'true'
os.environ['XLA_PYTHON_CLIENT_MEM_FRACTION'] = '.50'

from absl import app
from absl import flags
from absl import logging
from clu import platform
import jax
import tensorflow as tf
from frc971.control_loops.swerve import jax_dynamics

jax._src.deprecations.accelerate('tracer-hash')
# Enable the compilation cache
jax.config.update("jax_compilation_cache_dir", "/tmp/jax_cache")
jax.config.update("jax_persistent_cache_min_entry_size_bytes", -1)
jax.config.update("jax_persistent_cache_min_compile_time_secs", 0)
jax.config.update('jax_threefry_partitionable', True)

import train

FLAGS = flags.FLAGS

flags.DEFINE_string('workdir', None, 'Directory to store model data.')


def main(argv):
    if len(argv) > 1:
        raise app.UsageError('Too many command-line arguments.')

    # Hide any GPUs from TensorFlow. Otherwise it might reserve memory.
    tf.config.experimental.set_visible_devices([], 'GPU')

    logging.info('JAX process: %d / %d', jax.process_index(),
                 jax.process_count())
    logging.info('JAX local devices: %r', jax.local_devices())

    # Add a note so that we can tell which task is which JAX host.
    # (Depending on the platform task 0 is not guaranteed to be host 0)
    platform.work_unit().set_task_status(
        f'process_index: {jax.process_index()}, '
        f'process_count: {jax.process_count()}')
    platform.work_unit().create_artifact(platform.ArtifactType.DIRECTORY,
                                         FLAGS.workdir, 'workdir')

    logging.info(
        'Visualize results with: bazel run -c opt @pip_deps_tensorboard//:rules_python_wheel_entry_point_tensorboard -- --logdir %s',
        FLAGS.workdir,
    )

    physics_constants = jax_dynamics.Coefficients()
    state = train.train(FLAGS.workdir, physics_constants)


if __name__ == '__main__':
    flags.mark_flags_as_required(['workdir'])
    app.run(main)
