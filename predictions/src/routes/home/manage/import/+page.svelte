<script>
	import { goto } from '$app/navigation';
	import { error } from '@sveltejs/kit';
	import { doc, setDoc } from 'firebase/firestore';
	import { db } from '$lib/firestore';

	export let data;
	const { events } = data;

	let eventSelect = '';
	let urlInput = '';
	let errorMessage = '';

	async function importMatches() {
		if (eventSelect == '' || urlInput == '') {
			errorMessage = 'Select an event and enter a url';
			return;
		} else if (!urlInput.includes('bluealliance.com/event/')) {
			errorMessage = 'Enter a valid TBA url';
			return;
		} else {
			errorMessage = '';
		}

		const urlArr = urlInput.split('/');
		const eventCode = urlArr[urlArr.length - 1];
		console.log(eventCode);

		const url = 'https://www.thebluealliance.com/api/v3/event/' + eventCode + '/matches';

		let response = await fetch(url, {
			method: 'GET',
			headers: {
				'X-TBA-Auth-Key': '1JYHN4B6jREvErHpgG6uoaJ3dObX2oVp23QMv8KCfTZ3TFfS3Yw6nPWI1vItZEDk'
			}
		});
		let data = await response.json();

		console.log(data);

		for (let i = 0; i < data.length; i++) {
			const docID = eventSelect + '-' + data[i].match_number;
			const blue_team_array_raw = data[i].alliances.blue.team_keys;
			const blue_team_array = [];
			for (let b = 0; b < 3; b++) {
				blue_team_array.push(blue_team_array_raw[b].split('c')[1]);
			}
			const red_team_array_raw = data[i].alliances.red.team_keys;
			const red_team_array = [];
			for (let r = 0; r < 3; r++) {
				red_team_array.push(red_team_array_raw[r].split('c')[1]);
			}
			console.log(blue_team_array);
			console.log(red_team_array);

			await setDoc(doc(db, 'matches', docID), {
				active: true,
				event: eventSelect,
				match_finished: false,
				match_number: data[i].match_number,
				match_result: 'Red Alliance',
				blue_team: blue_team_array,
				red_team: red_team_array
			});

			goto('/home/manage');
		}
	}
</script>

<div class="grid h-screen place-items-center">
	<div class="card w-200 bg-base-300 shadow-xl">
		<div class="card-body">
			<div class="flex space-x-2">
				<button on:click={() => goto('/home/manage')} class="btn btn-square btn-xs btn-primary"
					>&#8592</button
				>
				<h2 class="card-title pb-7">Import</h2>
			</div>
			<div class="flex space-x-2">
				<div>
					<h4>Select Event</h4>
					<select bind:value={eventSelect} class="select select-bordered w-full max-w-xs">
						<option disabled selected>Event</option>
						{#each events as event}
							<option>{event.id}</option>
						{/each}
					</select>
				</div>
				<div>
					<h4>Event URL</h4>
					<input
						bind:value={urlInput}
						type="text"
						placeholder="Event URL"
						class="input input-bordered w-full max-w-xs"
					/>
				</div>
			</div>
			<button on:click={importMatches} class="btn btn-active btn-primary">Import</button>
			{#if errorMessage != ''}
				<h4>{errorMessage}</h4>
			{/if}
		</div>
	</div>
</div>
