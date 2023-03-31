<script>
	import { page } from '$app/stores';

	export let data;
	const { matches, match_id } = data;
	const ten_matches = matches.slice(0, 10);
</script>

<div class="grid h-screen place-items-center content-start md:content-center">
	<div class="md:card md:w-200 md:bg-base-300 md:shadow-xl">
		<div class="card-body">
			<div class="flex space-x-2 ">
				<a href="/home"><button class="btn btn-square btn-xs btn-primary">&#8592</button></a>
				<h2 class="card-title pb-7">Upcoming Matches</h2>
			</div>
			{#each ten_matches as match, index}
				<a class="relative" href="/home/events/{$page.params.slug}/{match_id[index]}">
					<div class="flex items-center space-x-2">
						<h3 class="text-black">#{match.match_number}</h3>
						<div class="btn-group">
							<button class="btn btn-square btn-error">{match.red_team[0]}</button>
							<button class="btn btn-square btn-error">{match.red_team[1]}</button>
							<button class="btn btn-square btn-error">{match.red_team[2]}</button>
							<button class="btn btn-square btn-accent">{match.blue_team[0]}</button>
							<button class="btn btn-square btn-accent">{match.blue_team[1]}</button>
							<button class="btn btn-square btn-accent">{match.blue_team[2]}</button>
						</div>
					</div>
				</a>
			{/each}
		</div>
	</div>
</div>
