/** @type {import('tailwindcss').Config} */
module.exports = {
	content: ['./src/**/*.{html,js,svelte,ts}'],
	theme: {
		extend: {}
	},
	plugins: [require('daisyui')],
	daisyui: {
		themes: [
			{
				mytheme: {
					primary: '#facc15',

					secondary: '#fde047',

					accent: '#38bdf8',

					neutral: '#72a8f8',

					'base-100': '#d6d6d6',

					info: '#14b8a6',

					success: '#22c55e',

					warning: '#FBBD23',

					error: '#F87272'
				}
			}
		]
	}
};
