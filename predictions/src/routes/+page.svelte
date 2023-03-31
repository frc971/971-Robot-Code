<script>
	import { robot, predict, podium, spartan, treasure } from '$lib/img/img';
</script>

<!--Navbar-->
<div class="pt-2">
	<div class="navbar bg-white ">
		<div class="navbar-start">
			<a class="btn btn-ghost normal-case text-xl text-black" href="/">Spartan Predictions</a>
		</div>
		<div class="navbar-end">
			<a class="btn btn-primary text-black" href="/login">Get started</a>
		</div>
	</div>
</div>

<!--Hero-->
<div class="hero min-h-screen">
	<div class="hero-content flex-col lg:flex-row">
		<div class="md:pr-36">
			<h1 class="text-7xl font-bold text-center md:text-left">Predict.<br />Compete.<br />Win.</h1>
			<p class="py-6 text-center md:text-left hidden md:block">An FRC971 App</p>
			<a href="/login"
				><div class="flex justify-center md:justify-start py-6 md:py-0">
					<button class="btn btn-primary">Get Started</button>
				</div></a
			>
		</div>
		<img alt="FRC Match" src={robot} class="max-w-xs md:max-w-2xl rounded-lg shadow-2xl" />
	</div>
</div>

<div class="flex flex-col space-y-12 md:space-y-0 md:space-x-24 md:flex-row place-content-center">
	<div>
		<h1
			class="text-8xl font-extrabold text-center text-transparent bg-clip-text bg-gradient-to-r from-yellow-500 to-pink-600"
		>
			"WOAH"
		</h1>
		<h2 class="text-3xl font-bold text-center pt-5">- Téa Fazio</h2>
	</div>
	<div>
		<h1
			class="text-8xl font-extrabold text-center text-transparent bg-clip-text bg-gradient-to-r from-green-500 to-blue-600"
		>
			"COOL"
		</h1>
		<h2 class="text-3xl font-bold text-center pt-5">- Kiran Haywood</h2>
	</div>
</div>

<div class="grid pt-52 pb-24 place-items-center">
	<div class="flex flex-col md:flex-row md:space-x-24 space-y-12 md:space-y-0">
		<div class="card w-96 bg-base-200 shadow-2xl">
			<figure><img src={predict} alt="UI Feature" /></figure>
			<div class="card-body">
				<h2 class="card-title">Place Predictions on Matches</h2>
				<p>Speculate on who you think will win the match</p>
			</div>
		</div>

		<div class="card w-96 bg-base-200 shadow-2xl">
			<figure class="bg-white"><img src={podium} alt="UI Feature" /></figure>
			<div class="card-body">
				<h2 class="card-title">Climb the Leaderboard</h2>
				<p>Compete with friends for the top spots on the leaderboard</p>
			</div>
		</div>

		<div class="card w-96 bg-base-200 shadow-2xl">
			<figure class="bg-white"><img src={treasure} alt="UI Feature" /></figure>
			<div class="card-body">
				<h2 class="card-title">Win Prizes!</h2>
				<p>Top competitors will win prizes (TBD)</p>
			</div>
		</div>
	</div>
</div>

<footer class="footer footer-center pt-10 pb-10 bg-primary text-primary-content">
	<div>
		<img width="50" height="50" src={spartan} alt="logo" />
		<p class="font-bold">
			Spartan Predictions <br />FRC 971
		</p>
		<p>Copyright © 2022 - All right reserved</p>
	</div>
	<div>
		<div class="grid grid-flow-col gap-4">
			<a href="https://www.instagram.com/frc971/" target="blank"
				><svg
					xmlns="http://www.w3.org/2000/svg"
					width="16"
					height="16"
					fill="currentColor"
					class="bi bi-instagram"
					viewBox="0 0 16 16"
				>
					<path
						d="M8 0C5.829 0 5.556.01 4.703.048 3.85.088 3.269.222 2.76.42a3.917 3.917 0 0 0-1.417.923A3.927 3.927 0 0 0 .42 2.76C.222 3.268.087 3.85.048 4.7.01 5.555 0 5.827 0 8.001c0 2.172.01 2.444.048 3.297.04.852.174 1.433.372 1.942.205.526.478.972.923 1.417.444.445.89.719 1.416.923.51.198 1.09.333 1.942.372C5.555 15.99 5.827 16 8 16s2.444-.01 3.298-.048c.851-.04 1.434-.174 1.943-.372a3.916 3.916 0 0 0 1.416-.923c.445-.445.718-.891.923-1.417.197-.509.332-1.09.372-1.942C15.99 10.445 16 10.173 16 8s-.01-2.445-.048-3.299c-.04-.851-.175-1.433-.372-1.941a3.926 3.926 0 0 0-.923-1.417A3.911 3.911 0 0 0 13.24.42c-.51-.198-1.092-.333-1.943-.372C10.443.01 10.172 0 7.998 0h.003zm-.717 1.442h.718c2.136 0 2.389.007 3.232.046.78.035 1.204.166 1.486.275.373.145.64.319.92.599.28.28.453.546.598.92.11.281.24.705.275 1.485.039.843.047 1.096.047 3.231s-.008 2.389-.047 3.232c-.035.78-.166 1.203-.275 1.485a2.47 2.47 0 0 1-.599.919c-.28.28-.546.453-.92.598-.28.11-.704.24-1.485.276-.843.038-1.096.047-3.232.047s-2.39-.009-3.233-.047c-.78-.036-1.203-.166-1.485-.276a2.478 2.478 0 0 1-.92-.598 2.48 2.48 0 0 1-.6-.92c-.109-.281-.24-.705-.275-1.485-.038-.843-.046-1.096-.046-3.233 0-2.136.008-2.388.046-3.231.036-.78.166-1.204.276-1.486.145-.373.319-.64.599-.92.28-.28.546-.453.92-.598.282-.11.705-.24 1.485-.276.738-.034 1.024-.044 2.515-.045v.002zm4.988 1.328a.96.96 0 1 0 0 1.92.96.96 0 0 0 0-1.92zm-4.27 1.122a4.109 4.109 0 1 0 0 8.217 4.109 4.109 0 0 0 0-8.217zm0 1.441a2.667 2.667 0 1 1 0 5.334 2.667 2.667 0 0 1 0-5.334z"
					/>
				</svg>
			</a>
			<a href="https://www.facebook.com/frc971" target="blank">
				<svg
					xmlns="http://www.w3.org/2000/svg"
					width="16"
					height="16"
					fill="currentColor"
					class="bi bi-facebook"
					viewBox="0 0 16 16"
				>
					<path
						d="M16 8.049c0-4.446-3.582-8.05-8-8.05C3.58 0-.002 3.603-.002 8.05c0 4.017 2.926 7.347 6.75 7.951v-5.625h-2.03V8.05H6.75V6.275c0-2.017 1.195-3.131 3.022-3.131.876 0 1.791.157 1.791.157v1.98h-1.009c-.993 0-1.303.621-1.303 1.258v1.51h2.218l-.354 2.326H9.25V16c3.824-.604 6.75-3.934 6.75-7.951z"
					/>
				</svg>
			</a>
		</div>
	</div>
</footer>
