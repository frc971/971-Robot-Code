<script>
	export let data;
	const { points } = data;

	let current_page = 0;
	let records_per_page = 10;

	function previous() {
		if (current_page > 0) {
			current_page -= records_per_page;
		}
	}

	function next() {
		if (current_page < points.length) {
			current_page += records_per_page;
		}
	}
</script>

<div class="grid h-screen place-items-center content-start md:content-center">
	<div class="md:card md:w-200 md:bg-base-300 md:shadow-xl">
		<div class="card-body">
			<h2 class="card-title pb-7">Leaderboard</h2>
			<table class="table w-full">
				<thead>
					<tr>
						<th />
						<th>Name</th>
						<th>Points</th>
						<!--<th>Win %</th>-->
					</tr>
				</thead>
				<tbody>
					{#each points.slice(current_page, current_page + records_per_page) as entry, index}
						<tr>
							<th>{index + 1 + current_page}</th>
							<th><h1>{entry.username}</h1></th>
							<th><h1>{entry.points.toString().replace(/\B(?=(\d{3})+(?!\d))/g, ',')}</h1></th>
							<!--<th><h1>{((entry.wins/entry.total_predictions)*100).toFixed(2)}%</h1></th>-->
						</tr>
					{/each}
				</tbody>
			</table>
			<div class="flex space-x-2">
				<btn class="btn btn-outline btn-md md:btn-wide" on:click={previous}>Previous</btn>
				<btn class="btn btn-outline btn-md md:btn-wide" on:click={next}>Next</btn>
			</div>
		</div>
	</div>
</div>
