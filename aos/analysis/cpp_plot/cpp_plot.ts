// Plotter for the C++ in-process plotter.
import {Configuration} from '../../../aos/configuration_generated';
import {Connection} from '../../../aos/network/www/proxy';
import {plotData} from '../plot_data_utils';

const rootDiv = document.createElement('div');
rootDiv.classList.add('aos_cpp_plot');
document.body.appendChild(rootDiv);

const conn = new Connection();
conn.connect();

conn.addConfigHandler((config: Configuration) => {
  plotData(conn, rootDiv);
});
