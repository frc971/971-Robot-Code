import { nodeResolve } from '@rollup/plugin-node-resolve';

export default {
  context: "window",
  plugins: [nodeResolve()]
};
