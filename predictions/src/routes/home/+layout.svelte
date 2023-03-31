<script lang="ts">
	import authStore from '../../lib/authStore';
	import { getAuth, signOut } from 'firebase/auth';
	import { goto } from '$app/navigation';
	import { onMount } from 'svelte';
	import { uid, username } from '$lib/store';
	import { auth } from '$lib/firestore';

	export let data;
	const { admin, points } = data;
	let user_points = points.points;

	let user_id: string;
	uid.subscribe((value) => {
		user_id = value;
	});

	authStore.subscribe(async ({ isLoggedIn, firebaseControlled }) => {
		if (!isLoggedIn && firebaseControlled) {
			await goto('/');
		}
	});

	async function logOut() {
		signOut(auth)
			.then(() => {
				// Sign-out successful.
			})
			.catch((error) => {
				// An error happened.
			});
	}

	function manage() {
		goto('/home/manage');
	}
	function leaderboard() {
		goto('/home/leaderboard');
	}
	function predictions() {
		const uid = localStorage.getItem('uid');
		goto('/home/predictions/' + uid);
	}
</script>

<body style="">
	<div class="">
		<div class="navbar bg-secondary">
			<div class="navbar-start">
				<a class="btn btn-ghost normal-case text-xl text-black" href="/home"
					>Spartan Predictions {user_id}</a
				>
			</div>
			<div class="flex justify-end flex-1 px-2">
				<div class="flex items-stretch">
					<div class="dropdown dropdown-end">
						<!--Comment to ignore warnings-->
						<!-- svelte-ignore a11y-no-noninteractive-tabindex -->
						<!-- svelte-ignore a11y-label-has-associated-control -->
						<label tabindex="0" class="btn btn-ghost rounded-btn">&#9660</label>
						<!-- svelte-ignore a11y-no-noninteractive-tabindex -->
						<!-- svelte-ignore a11y-label-has-associated-control -->
						<ul
							tabindex="0"
							class="menu dropdown-content p-4 shadow bg-base-100 rounded-box w-52 mt-4 space-y-2"
						>
							{#if admin}
								<li><button on:click={manage} class="btn btn-outline">Manage</button></li>
							{/if}
							<li><button class="btn btn-outline">Balance: ${user_points}</button></li>
							<li><button on:click={predictions} class="btn btn-outline">Predictions</button></li>
							<li><button on:click={leaderboard} class="btn btn-outline">Leaderboard</button></li>
							<li><button on:click={logOut} class="btn btn-outline">Log Out</button></li>
						</ul>
					</div>
				</div>
			</div>
		</div>
		<slot />
	</div></body
>
