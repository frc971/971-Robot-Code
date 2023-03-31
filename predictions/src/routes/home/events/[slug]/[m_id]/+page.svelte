<script>
	import { error } from '@sveltejs/kit';
	import { db } from '$lib/firestore';
	import { getDoc, doc, setDoc, updateDoc } from 'firebase/firestore';

	export let data;
	const { match, match_id } = data;

	let predictionSelect = '';
	let betInput = '1';
	const betValues = [100, 500, 1000, 2500, 5000, 10000];
	$: betValue = betValues[parseInt(betInput)];
	let selectionError = false;
	let activeEventError = false;
	let balanceError = false;

	function goBack() {
		history.back();
	}

	async function submitSelection() {
		if (predictionSelect == '') {
			selectionError = true;
		} else {
			const uid = localStorage.getItem('uid');

			//Check if user has enough credits to place bet
			const userRef = doc(db, 'points', uid);
			const userSnap = await getDoc(userRef);
			const userData = userSnap.data();

			if (userData) {
				if (userData.points < betValue) {
					balanceError = true;
					return;
				}
			} else {
				balanceError = true;
				return;
			}

			//Unique doc id
			const doc_id = match_id + '-' + uid;

			//Fixes issue #3
			//Checks if the match is still active and stops users
			//from placing predictions if it is NOT active
			const docRef = doc(db, 'matches', match_id);
			const docSnap = await getDoc(docRef);

			if (docSnap.exists()) {
				const activeBool = docSnap.data().active;

				if (!activeBool) {
					activeEventError = true;
					return;
				}
			} else {
				activeEventError = true;
				return;
			}

			goBack();

			//Check if user has already placed a prediction
			// and is only updating/editing it
			// so the user dosen't spend multiple times on the same bet
			const pdocref = doc(db, 'predictions', doc_id);
			const pdocSnap = await getDoc(pdocref);

			if (pdocSnap.exists()) {
				//USER IS EDITING OLD PREDICTION
				console.log('exists');

				const pdocData = pdocSnap.data();

				const oldBetValue = pdocData.amount;

				if (oldBetValue > betValue) {
					//User DECREASED bet amount
					//refund difference
					const refundDiff = oldBetValue - betValue;

					await updateDoc(userRef, {
						points: userData.points + refundDiff
					});
				} else if (oldBetValue < betValue) {
					//User INCREASED bet amount
					//refund difference
					const refundDiff = betValue - oldBetValue;
					await updateDoc(userRef, {
						points: userData.points - refundDiff
					});
				}

				//Overwrite the old prediction doc
				await setDoc(doc(db, 'predictions', doc_id), {
					match: match_id,
					prediction: predictionSelect,
					amount: betValue,
					uid: uid
				});
			} else {
				//USER IS CREATING A NEW PREDICTION
				console.log('no exists');

				//Deduct points from user balance
				await updateDoc(userRef, {
					points: userData.points - betValue
				});

				//Create a new prediction doc
				await setDoc(doc(db, 'predictions', doc_id), {
					match: match_id,
					prediction: predictionSelect,
					amount: betValue,
					uid: uid
				});
			}
		}
	}
</script>

<div class="grid h-screen place-items-center">
	<div class="card w-200 bg-base-300 shadow-xl">
		<div class="card-body">
			<div class="flex space-x-2 ">
				<button on:click={goBack} class="btn btn-square btn-xs btn-primary">&#8592</button>
				<h2 class="card-title pb-7">Match #{match.match_number}</h2>
			</div>
			<div class="btn-group">
				<button class="btn btn-square btn-error">{match.red_team[0]}</button>
				<button class="btn btn-square btn-error">{match.red_team[1]}</button>
				<button class="btn btn-square btn-error">{match.red_team[2]}</button>
				<button class="btn btn-square btn-accent">{match.blue_team[0]}</button>
				<button class="btn btn-square btn-accent">{match.blue_team[1]}</button>
				<button class="btn btn-square btn-accent">{match.blue_team[2]}</button>
			</div>
			<div class="">
				<select bind:value={predictionSelect} class="select select-bordered w-full max-w-xs">
					<option disabled selected>Who will win?</option>
					<option>Red Alliance</option>
					<option>Blue Alliance</option>
				</select>
				<div class="pt-5">
					<input
						bind:value={betInput}
						type="range"
						min="0"
						max="5"
						class="range range-primary"
						step="1"
					/>
					<div class="w-full flex justify-between text-xs px-2">
						<span>100</span>
						<span>500</span>
						<span>1000</span>
						<span>2500</span>
						<span>5,000</span>
						<span>10,000</span>
					</div>
				</div>
				<div class="pt-2">
					<button on:click={submitSelection} class="btn btn-primary">Submit</button>
					{#if selectionError}<p>Select an option</p>{/if}
					{#if activeEventError}<p>Error: This match is not active</p>{/if}
					{#if balanceError}<p>Error getting user balance or user balance insufficient</p>{/if}
				</div>
			</div>
		</div>
	</div>
</div>
