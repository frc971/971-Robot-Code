import { getFirestore } from 'firebase/firestore';
import { initializeApp } from 'firebase/app';
import { getAuth } from 'firebase/auth';

const firebaseConfig = {
	apiKey: '',
	authDomain: 'spartan-predictions.firebaseapp.com',
	projectId: 'spartan-predictions',
	storageBucket: 'spartan-predictions.appspot.com',
	messagingSenderId: '541826816726',
	appId: '1:541826816726:web:4eb4bc915fd882c7b6c777',
	measurementId: 'G-1Q4X7F0XTJ'
};

export const app = initializeApp(firebaseConfig);
export const auth = getAuth(app);
export const db = getFirestore(app);
