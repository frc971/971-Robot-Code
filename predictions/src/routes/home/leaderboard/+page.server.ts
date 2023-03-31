import { error } from '@sveltejs/kit';
import { db } from '$lib/firestore';
import { collection, getDocs } from 'firebase/firestore';
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
	const eventsSnapshot = await getDocs(collection(db, 'points'));

	let points: DocumentData[] = [];
	eventsSnapshot.forEach((doc) => {
		points.push(doc.data());
	});

	points = points.sort(sort_by('points', true, parseInt));

	return {
		points: points
	};
}
