// Plotter for the C++ in-process plotter.
import {Configuration} from 'org_frc971/aos/configuration_generated';
import {Connection} from 'org_frc971/aos/network/www/proxy';
import {plotData} from 'org_frc971/frc971/analysis/plot_data_utils';

const rootDiv = document.createElement('div');
rootDiv.classList.add('aos_cpp_plot');
document.body.appendChild(rootDiv);

const conn = new Connection();
conn.connect();

conn.addConfigHandler((config: Configuration) => {
  plotData(conn, rootDiv);
});
