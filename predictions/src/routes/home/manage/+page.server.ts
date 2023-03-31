import { error } from '@sveltejs/kit';
import { db } from '$lib/firestore';
import { collection, query, where, getDocs, doc, getDoc } from 'firebase/firestore';
import type { DocumentData } from 'firebase/firestore';

const sort_by = (field, reverse, primer) => {
	const key = primer
		? function (x) {
				return primer(x[field]);
		  }
		: function (x) {
				return x[field];
		  };

	reverse = !reverse ? 1 : -1;

	return function (a, b) {
		return (a = key(a)), (b = key(b)), reverse * ((a > b) - (b > a));
	};
};

/** @type {import('./$types').PageServerLoad} */
export async function load({ params }) {
	//Query Events
	const eventsq = query(collection(db, 'events'));
	const eventsSnapshot = await getDocs(eventsq);

	let events: DocumentData[] = [];
	eventsSnapshot.forEach((doc) => {
		events.push(doc.data());
	});

	const matchesq = query(collection(db, 'matches'), where('match_finished', '==', false));
	const matchesSnapshot = await getDocs(matchesq);

	let matches: DocumentData[] = [];

	matchesSnapshot.forEach((doc) => {
		matches.push(doc.data());
	});

	matches = matches.sort(sort_by('match_number', false, parseInt));

	return {
		events: events,
		matches: matches
	};
}
