<script lang="ts">
	import { getAuth, signInWithPopup, GoogleAuthProvider } from 'firebase/auth';
	import { doc, getDoc, setDoc } from 'firebase/firestore';
	import { db } from '$lib/firestore';
	import { goto } from '$app/navigation';
	import { google } from '$lib/img/img';

	async function createPointsRecord(uid: string, username: string) {
		//Check if doc has not been created
		const docRef = doc(db, 'points', uid);
		const docSnap = await getDoc(docRef);
		if (docSnap.exists()) {
		} else {
			await setDoc(doc(db, 'points', uid), {
				uid: uid,
				points: 1000,
				total_predictions: 0,
				wins: 0,
				losses: 0,
				username: username
			});
		}
	}

	async function loginWithGoogle() {
		try {
			const provider = new GoogleAuthProvider();

			const auth = getAuth();
			signInWithPopup(auth, provider)
				.then((result) => {
					const credential = GoogleAuthProvider.credentialFromResult(result);
					const token = credential.accessToken;
					const user = result.user;
					localStorage.setItem('uid', user.uid);
					createPointsRecord(user.uid, user.displayName);

					goto('/home');
				})
				.catch((error) => {
					// Handle Errors here.
					const errorCode = error.code;
					const errorMessage = error.message;
					console.log(errorMessage);
				});
		} catch (e) {
			console.log(e);
		}
	}
</script>

<svelte:head>
	<script src="https://apis.google.com/js/platform.js" async defer></script>
</svelte:head>

<div class="grid h-screen place-items-center">
	<div class="">
		<div class="pt-10">
			<button on:click={loginWithGoogle} class="btn btn-outline text-black">
				<img src={google} alt="" width="30" class="pr-2" />
				Sign in with Google
			</button>
		</div>
	</div>
</div>
