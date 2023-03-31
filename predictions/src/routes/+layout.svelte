<script>
	import '../app.css';

	import { getAuth, onAuthStateChanged } from 'firebase/auth';
	import authStore from '$lib/authStore';
	import { uid, username } from '$lib/store';
	import { onMount } from 'svelte';
	import { app } from '$lib/firestore';
	import { goto } from '$app/navigation';
	import { inject } from '@vercel/analytics';

	onMount(() => {
		const auth = getAuth(app);
		onAuthStateChanged(auth, (user) => {
			if (!user) {
				goto('/');
			} else {
				uid.update((uid) => user.uid);
				username.update((username) => user.displayName);
			}
			authStore.set({
				isLoggedIn: user !== null,
				user,
				firebaseControlled: true
			});
		});
	});
</script>

<body class="overflow-x-hidden">
	<div class="min-h-screen bg-white ">
		<slot />
	</div>
</body>
