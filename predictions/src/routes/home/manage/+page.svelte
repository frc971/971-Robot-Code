<script>
	import { doc, updateDoc, collection, query, where, getDocs, getDoc } from 'firebase/firestore';
	import { db } from '$lib/firestore';
	import { onMount } from 'svelte';
	import { goto } from '$app/navigation';

	export let data;
	const { events, matches } = data;

	//Pagination
	let current_page = 1;
	let records_per_page = 5;

	let filteredMatches = matches;

	function pageUp() {
		if (current_page < Math.ceil(filteredMatches.length / records_per_page)) {
			current_page++;
		}
	}
	function pageDown() {
		if (current_page > 1) {
			current_page--;
		}
	}

	let eventSelect = 'None';

	onMount(() => {
		if (localStorage.getItem('eventSelect') == null) {
			localStorage.setItem('eventSelect', 'None');
		} else {
			eventSelect = localStorage.getItem('eventSelect');
			console.log('mount', eventSelect, matches);
			const eventMatches = matches.filter((match) => match.event === eventSelect);
			filteredMatches = eventMatches.filter((ematch) => ematch.match_finished != true);
			console.log(filteredMatches);
		}
	});

	function handleEventSelect() {
		localStorage.setItem('eventSelect', eventSelect);
		console.log('handle', eventSelect, matches);
		const eventMatches = matches.filter((match) => match.event === eventSelect);
		filteredMatches = eventMatches.filter((ematch) => ematch.match_finished != true);
		console.log(filteredMatches);
		pagination = 0;
	}

	async function handleActivate(match_number) {
		const docID = eventSelect + '-' + match_number;
		const matchRef = doc(db, 'matches', docID);
		await updateDoc(matchRef, {
			active: true
		});
		location.reload();
	}

	async function handleDeactivate(match_number) {
		const docID = eventSelect + '-' + match_number;
		const matchRef = doc(db, 'matches', docID);
		await updateDoc(matchRef, {
			active: false
		});
		location.reload();
	}

	async function handleResultRed(match_number) {
		const docID = eventSelect + '-' + match_number;
		const matchRef = doc(db, 'matches', docID);
		await updateDoc(matchRef, {
			match_result: 'Red Alliance'
		});
		location.reload();
	}

	async function handleResultBlue(match_number) {
		const docID = eventSelect + '-' + match_number;
		const matchRef = doc(db, 'matches', docID);
		await updateDoc(matchRef, {
			match_result: 'Blue Alliance'
		});
		location.reload();
	}

	async function handleSubmit(match_number) {
		const docID = eventSelect + '-' + match_number;
		const matchRef = doc(db, 'matches', docID);
		await updateDoc(matchRef, {
			match_finished: true
		});

		//first get correct result
		const docSnap = await getDoc(matchRef);
		const match_result = docSnap.data().match_result;

		//get all users who got prediction right/wrong
		const q = query(collection(db, 'predictions'), where('match', '==', docID));
		let uid_correct = [];
		let uid_wrong = [];

		const querySnapshot = await getDocs(q);
		querySnapshot.forEach((doc) => {
			if (doc.data().prediction == match_result) {
				uid_correct.push([doc.data().uid, doc.data().amount]);
			} else {
				uid_wrong.push(doc.data().uid);
			}
		});

		console.log('giving winner points', uid_correct);
		//Give correct users 10 points
		if (uid_correct.length > 0) {
			for (let i = 0; i < uid_correct.length; i++) {
				let pointsRef = doc(db, 'points', uid_correct[i][0]);
				let docSnap = await getDoc(pointsRef);
				let old_points = docSnap.data().points;
				let old_wins = docSnap.data().wins;
				let old_total_predictions = docSnap.data().total_predictions;

				await updateDoc(pointsRef, {
					points: old_points + uid_correct[i][1] + uid_correct[i][1],
					wins: old_wins + 1,
					total_predictions: old_total_predictions + 1
				});
			}
		}
		//Give wrong users -10 points
		console.log('giving losser points', uid_wrong);
		if (uid_wrong.length > 0) {
			for (let i = 0; i < uid_wrong.length; i++) {
				let pointsRef = doc(db, 'points', uid_wrong[i]);
				let docSnap = await getDoc(pointsRef);
				if (!docSnap.data()) {
					console.log('no doc data');
				}
				let old_losses = docSnap.data().losses;
				let old_total_predictions = docSnap.data().total_predictions;

				await updateDoc(pointsRef, {
					losses: old_losses + 1,
					total_predictions: old_total_predictions + 1
				});
			}
		}

		location.reload();
	}
</script>

<div class="grid h-screen place-items-center">
	<div class="grid h-screen place-items-center">
		<div class="card w-200 bg-base-300 shadow-xl">
			<div class="card-body">
				<div class="flex space-x-2">
					<h2 class="card-title pb-7">Manage Matches</h2>
				</div>
				<div class="flex space-x-10">
					<select
						bind:value={eventSelect}
						on:change={handleEventSelect}
						class="select select-bordered w-full max-w-xs"
					>
						<option disabled selected>Select Event</option>
						{#each events as event}
							<option>{event.id}</option>
						{/each}
					</select>
					<button on:click={() => goto('/home/manage/create')} class="btn btn-primary"
						>Create Match</button
					>
					<button on:click={() => goto('/home/manage/import')} class="btn btn-primary"
						>Import Matches</button
					>
					<button on:click={() => goto('/home/manage/help')} class="btn btn-accent">Help</button>
				</div>
				<br />
				<table class="table w-full">
					<thead>
						<tr>
							<th>#</th>
							<th>Teams</th>
							<th>Active</th>
							<th>Result (Red Win)</th>
							<th>Submit (Send Points)</th>
						</tr>
					</thead>
					{#if eventSelect != 'None'}
						<tbody>
							{#each filteredMatches.slice((current_page - 1) * records_per_page, current_page * records_per_page) as match}
								<tr>
									<th><h3 class="text-black">{match.match_number}</h3></th>
									<th
										><div class="btn-group">
											<button class="btn btn-square btn-error">{match.red_team[0]}</button>
											<button class="btn btn-square btn-error">{match.red_team[1]}</button>
											<button class="btn btn-square btn-error">{match.red_team[2]}</button>
											<button class="btn btn-square btn-accent">{match.blue_team[0]}</button>
											<button class="btn btn-square btn-accent">{match.blue_team[1]}</button>
											<button class="btn btn-square btn-accent">{match.blue_team[2]}</button>
										</div></th
									>
									{#if match.active}
										<th
											><input
												on:click={() => handleDeactivate(match.match_number)}
												type="checkbox"
												checked="checked"
												class="checkbox"
											/></th
										>
									{:else}
										<th
											><input
												on:click={() => handleActivate(match.match_number)}
												type="checkbox"
												class="checkbox"
											/></th
										>
									{/if}
									<!--Result-->
									<th>
										{#if match.active}
											<select class="select w-full max-w-xs" disabled>
												<option>Match still active</option>
											</select>
										{:else if match.match_result == 'Red Alliance' || match.match_result == ''}
											<input
												on:click={() => handleResultBlue(match.match_number)}
												type="checkbox"
												checked="checked"
												class="checkbox checkbox-error"
											/>
										{:else}
											<input
												on:click={() => handleResultRed(match.match_number)}
												type="checkbox"
												class="checkbox checkbox-error"
											/>
										{/if}
									</th>

									<!--Submit-->

									<th>
										{#if match.active}
											<button
												on:click={() => handleSubmit(match.match_number)}
												class="btn btn-primary btn-disabled">Submit</button
											>
										{:else}
											<button
												on:click={() => handleSubmit(match.match_number)}
												class="btn btn-primary">Submit</button
											>
										{/if}
									</th>
								</tr>
							{/each}
						</tbody>
						<br />
						<div class="flex space-x-2">
							{#if current_page == 1}
								<button on:click={pageDown} class="btn btn-outline btn-disabled">Back</button>
							{:else}
								<button on:click={pageDown} class="btn btn-outline">Back</button>
							{/if}

							{#if current_page == Math.ceil(filteredMatches.length / records_per_page)}
								<button on:click={pageUp} class="btn btn-outline btn-disabled">Next</button>
							{:else}
								<button on:click={pageUp} class="btn btn-outline">Next</button>
							{/if}
						</div>
					{/if}
				</table>
			</div>
		</div>
	</div>
</div>
